#include "camera.hpp"

#include <MvCameraControl.h>

#include <algorithm>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace {

void check(int status, const std::string& action, bool retryable = false) {
    if (status == MV_OK) return;
    std::ostringstream message;
    message << action << " failed (MVS 0x" << std::hex << static_cast<std::uint32_t>(status) << ')';
    if (retryable) throw CameraUnavailable(message.str());
    throw std::runtime_error(message.str());
}

MV_CC_DEVICE_INFO_LIST enumerate() {
    MV_CC_DEVICE_INFO_LIST devices{};
    check(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices), "enumerate cameras", true);
    return devices;
}

template <std::size_t Size>
std::string field(const unsigned char (&value)[Size]) {
    return std::string(value, std::find(value, value + Size, 0));
}

std::string serial(const MV_CC_DEVICE_INFO& device) {
    if (device.nTLayerType == MV_GIGE_DEVICE) return field(device.SpecialInfo.stGigEInfo.chSerialNumber);
    return field(device.SpecialInfo.stUsb3VInfo.chSerialNumber);
}

class Session {
public:
    ~Session() {
        if (grabbing) MV_CC_StopGrabbing(handle);
        if (opened) MV_CC_CloseDevice(handle);
        if (handle) MV_CC_DestroyHandle(handle);
    }

    void* handle = nullptr;
    bool opened = false;
    bool grabbing = false;
};

struct Buffer {
    void* handle;
    MV_FRAME_OUT frame{};
    bool acquired = false;

    ~Buffer() {
        if (acquired) MV_CC_FreeImageBuffer(handle, &frame);
    }
};

class MvsCamera final : public Camera {
public:
    MvsCamera(MV_CC_DEVICE_INFO* device, std::string serial_number, const std::string& trigger,
              std::uint64_t device_clock_hz, const std::string& profile, AutoExposure auto_exposure)
        : serial_(std::move(serial_number)), device_clock_hz_(device_clock_hz) {
        check(MV_CC_CreateHandle(&session_.handle, device), serial_ + ": create handle", true);
        check(MV_CC_OpenDevice(session_.handle, MV_ACCESS_Exclusive, 0), serial_ + ": open camera", true);
        session_.opened = true;
        if (!profile.empty()) {
            check(MV_CC_FeatureLoad(session_.handle, profile.c_str()), serial_ + ": apply camera profile");
        }

        if (device_clock_hz_) {
            if (device->nTLayerType != MV_GIGE_DEVICE) throw std::runtime_error("PTP mode requires GigE cameras");
            MVCC_INTVALUE_EX frequency{};
            check(MV_CC_GetIntValueEx(session_.handle, "GevTimestampTickFrequency", &frequency), serial_ + ": read tick frequency");
            if (frequency.nCurValue <= 0 || static_cast<std::uint64_t>(frequency.nCurValue) != device_clock_hz_) {
                throw std::runtime_error(serial_ + ": tick frequency differs from --device-clock-hz");
            }
            bool ptp_enabled = false;
            check(MV_CC_GetBoolValue(session_.handle, "GevIEEE1588", &ptp_enabled), serial_ + ": read PTP configuration");
            if (!ptp_enabled) throw std::runtime_error(serial_ + ": enable PTP before using device timestamps");
        }

        if (trigger.empty()) {
            check(MV_CC_SetEnumValueByString(session_.handle, "TriggerMode", "Off"), serial_ + ": free run");
        } else {
            check(MV_CC_SetEnumValueByString(session_.handle, "TriggerSelector", "FrameStart"), serial_ + ": FrameStart");
            check(MV_CC_SetEnumValueByString(session_.handle, "TriggerMode", "On"), serial_ + ": enable trigger");
            check(MV_CC_SetEnumValueByString(session_.handle, "TriggerSource", trigger.c_str()), serial_ + ": trigger source");
        }
        const char* mode = nullptr;
        switch (auto_exposure) {
            case AutoExposure::profile: break;
            case AutoExposure::off: mode = "Off"; break;
            case AutoExposure::once: mode = "Once"; break;
            case AutoExposure::continuous: mode = "Continuous"; break;
        }
        if (mode) {
            check(MV_CC_SetEnumValueByString(session_.handle, "ExposureAuto", mode), serial_ + ": set ExposureAuto");
        }
        if (device->nTLayerType == MV_USB_DEVICE) {
            check(MV_CC_SetGrabStrategy(session_.handle, MV_GrabStrategy_LatestImagesOnly), serial_ + ": set grab strategy");
        }
    }

    std::optional<Frame> read() override {
        if (device_clock_hz_ && Clock::now() >= next_ptp_check_) check_ptp_status();
        if (!session_.grabbing) {
            check(MV_CC_StartGrabbing(session_.handle), serial_ + ": start grabbing", true);
            session_.grabbing = true;
        }
        Buffer buffer{session_.handle};
        const auto status = MV_CC_GetImageBuffer(session_.handle, &buffer.frame, 200);
        const auto received = Clock::now();
        if (static_cast<std::uint32_t>(status) == static_cast<std::uint32_t>(MV_E_NODATA)) return std::nullopt;
        check(status, serial_ + ": get image", true);
        buffer.acquired = true;

        const auto& info = buffer.frame.stFrameInfo;
        const auto ticks = (static_cast<std::uint64_t>(info.nDevTimeStampHigh) << 32) | info.nDevTimeStampLow;
        const auto sequence = info.nFrameNum;
        if (info.nLostPacket) return Frame{{}, received, ticks, sequence, info.nLostPacket};
        const auto pixels = static_cast<std::uint64_t>(info.nWidth) * info.nHeight;
        if (info.nWidth == 0 || info.nHeight == 0 || info.nWidth > 8192 || info.nHeight > 8192 ||
            pixels > 16777216 || !buffer.frame.pBufAddr || info.nFrameLen == 0) {
            throw std::runtime_error(serial_ + ": unsupported or empty camera frame");
        }

        Image image{static_cast<int>(info.nWidth), static_cast<int>(info.nHeight),
                    std::vector<std::uint8_t>(pixels * 3)};
        if (info.enPixelType == PixelType_Gvsp_BGR8_Packed && info.nFrameLen == image.bgr.size()) {
            std::copy_n(buffer.frame.pBufAddr, image.bgr.size(), image.bgr.data());
        } else if (info.enPixelType == PixelType_Gvsp_Mono8 && info.nFrameLen == pixels) {
            for (std::size_t i = 0; i < pixels; ++i) {
                std::fill_n(image.bgr.data() + i * 3, 3, buffer.frame.pBufAddr[i]);
            }
        } else {
            MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
            conversion.nWidth = info.nWidth;
            conversion.nHeight = info.nHeight;
            conversion.enSrcPixelType = info.enPixelType;
            conversion.pSrcData = buffer.frame.pBufAddr;
            conversion.nSrcDataLen = info.nFrameLen;
            conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            conversion.pDstBuffer = image.bgr.data();
            conversion.nDstBufferSize = static_cast<unsigned int>(image.bgr.size());
            check(MV_CC_ConvertPixelTypeEx(session_.handle, &conversion), serial_ + ": convert image to BGR8");
            if (conversion.nDstLen != image.bgr.size()) {
                throw std::runtime_error(serial_ + ": unexpected converted image length");
            }
        }
        const auto release = MV_CC_FreeImageBuffer(session_.handle, &buffer.frame);
        buffer.acquired = false;
        check(release, serial_ + ": release image", true);
        return Frame{std::move(image), received, ticks, sequence, 0};
    }

private:
    void check_ptp_status() {
        MVCC_ENUMVALUE status{};
        check(MV_CC_GetEnumValue(session_.handle, "GevIEEE1588Status", &status), serial_ + ": read PTP state", true);
        MVCC_ENUMENTRY entry{};
        entry.nValue = status.nCurValue;
        check(MV_CC_GetEnumEntrySymbolic(session_.handle, "GevIEEE1588Status", &entry), serial_ + ": decode PTP state", true);
        const std::string state(entry.chSymbolic, std::find(std::begin(entry.chSymbolic), std::end(entry.chSymbolic), '\0'));
        if (state != "Master" && state != "Slave") throw CameraUnavailable(serial_ + ": PTP state=" + state);
        next_ptp_check_ = Clock::now() + std::chrono::seconds(1);
    }

    std::string serial_;
    std::uint64_t device_clock_hz_;
    Clock::time_point next_ptp_check_{};
    Session session_;
};

}  // namespace

MvsRuntime::MvsRuntime() { check(MV_CC_Initialize(), "initialize MVS"); }
MvsRuntime::~MvsRuntime() { MV_CC_Finalize(); }

void list_mvs_cameras() {
    const auto devices = enumerate();
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
        const auto* device = devices.pDeviceInfo[i];
        if (!device) continue;
        const bool gige = device->nTLayerType == MV_GIGE_DEVICE;
        const auto model = gige ? field(device->SpecialInfo.stGigEInfo.chModelName)
                                : field(device->SpecialInfo.stUsb3VInfo.chModelName);
        std::cout << serial(*device) << '\t' << (gige ? "GigE" : "USB3 Vision") << '\t' << model << '\n';
    }
}

std::unique_ptr<Camera> open_mvs_camera(const std::string& serial_number, const std::string& trigger,
                                       std::uint64_t device_clock_hz, const std::string& profile,
                                       AutoExposure auto_exposure) {
    static std::mutex enumeration;
    std::lock_guard<std::mutex> lock(enumeration);
    const auto devices = enumerate();
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
        auto* device = devices.pDeviceInfo[i];
        if (device && serial(*device) == serial_number) {
            return std::make_unique<MvsCamera>(device, serial_number, trigger, device_clock_hz, profile, auto_exposure);
        }
    }
    throw CameraUnavailable("camera serial not found: " + serial_number);
}
