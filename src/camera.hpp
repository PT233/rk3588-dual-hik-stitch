#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;

struct Image {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> bgr;
};

struct Frame {
    Image image;
    Clock::time_point received;
    std::uint64_t device_ticks = 0;
    std::optional<std::uint32_t> sequence;
    std::uint32_t lost_packets = 0;
    std::optional<std::uint32_t> source_pixel_type{};
};

class CameraUnavailable : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Camera {
public:
    virtual ~Camera() = default;
    virtual std::optional<Frame> read() = 0;
};

enum class AutoExposure { profile, off, once, continuous };

#ifdef ENABLE_MVS
class MvsRuntime {
public:
    MvsRuntime();
    ~MvsRuntime();
    MvsRuntime(const MvsRuntime&) = delete;
    MvsRuntime& operator=(const MvsRuntime&) = delete;
};

void list_mvs_cameras();
std::unique_ptr<Camera> open_mvs_camera(const std::string& serial, const std::string& trigger,
                                       std::uint64_t device_clock_hz, const std::string& profile,
                                       AutoExposure auto_exposure);
#endif
