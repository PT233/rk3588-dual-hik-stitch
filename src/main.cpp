#include "camera.hpp"
#include "pipeline.hpp"
#include "raw_output.hpp"
#include "stitch.hpp"
#include "stream_control.h"

#include <charconv>
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>

namespace {

using namespace std::chrono_literals;
volatile std::sig_atomic_t interrupted = 0;

void interrupt(int) { interrupted = 1; }

struct Options {
    bool synthetic = false;
    bool list = false;
    bool require_bound_calibration = false;
    std::string left, right, trigger, left_profile, right_profile;
    std::filesystem::path calibration, snapshot, output;
    int fps = 30;
    int frames = 0;
    int stitch_threads = 1;
    std::chrono::milliseconds write_timeout{2000};
    CapturePolicy capture;
};

void usage(std::ostream& out) {
    out << "dual-hik-stitch --list\n"
           "dual-hik-stitch --left SERIAL --right SERIAL --snapshot PREFIX\n"
           "dual-hik-stitch (--synthetic | --left SERIAL --right SERIAL) --calibration FILE\n"
           "  [--frames N] [--fps N (synthetic source)] [--max-skew-ms N | --max-skew-us N]\n"
           "  [--trigger Line0] [--device-clock-hz N (PTP GigE)] [--require-bound-calibration]\n"
           "  [--left-profile FILE] [--right-profile FILE] (MVS feature files)\n"
           "  [--camera-timeout-ms N] [--pair-timeout-ms N] [--write-timeout-ms N (Linux)]\n"
           "  [--stitch-threads N] [--output FILE (default: raw BGR24 on stdout)]\n";
}

int number(std::string_view value, int minimum, int maximum) {
    int result = 0;
    const auto* end = value.data() + value.size();
    const auto parsed = std::from_chars(value.data(), end, result);
    if (parsed.ec != std::errc{} || parsed.ptr != end || result < minimum || result > maximum) {
        throw std::runtime_error("invalid numeric option: " + std::string(value));
    }
    return result;
}

Options parse(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view flag = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc || std::string_view(argv[i]).substr(0, 2) == "--") {
                throw std::runtime_error("missing value for " + std::string(flag));
            }
            return argv[i];
        };
        if (flag == "--synthetic") options.synthetic = true;
        else if (flag == "--list") options.list = true;
        else if (flag == "--require-bound-calibration") options.require_bound_calibration = true;
        else if (flag == "--left") options.left = value();
        else if (flag == "--right") options.right = value();
        else if (flag == "--left-profile") options.left_profile = value();
        else if (flag == "--right-profile") options.right_profile = value();
        else if (flag == "--trigger") options.trigger = value();
        else if (flag == "--calibration") options.calibration = value();
        else if (flag == "--snapshot") options.snapshot = value();
        else if (flag == "--output") options.output = value();
        else if (flag == "--fps") options.fps = number(value(), 1, 240);
        else if (flag == "--frames") options.frames = number(value(), 1, 10000000);
        else if (flag == "--max-skew-ms") options.capture.max_skew = std::chrono::milliseconds(number(value(), 1, 1000));
        else if (flag == "--max-skew-us") options.capture.max_skew = std::chrono::microseconds(number(value(), 1, 1000000));
        else if (flag == "--camera-timeout-ms") options.capture.camera_timeout = std::chrono::milliseconds(number(value(), 200, 300000));
        else if (flag == "--pair-timeout-ms") options.capture.no_pair_timeout = std::chrono::milliseconds(number(value(), 200, 300000));
        else if (flag == "--write-timeout-ms") options.write_timeout = std::chrono::milliseconds(number(value(), 10, 30000));
        else if (flag == "--device-clock-hz") options.capture.device_clock_hz = number(value(), 1, 2000000000);
        else if (flag == "--stitch-threads") options.stitch_threads = number(value(), 1, 8);
        else throw std::runtime_error("unknown option: " + std::string(flag));
    }
    if (options.list && argc != 2) throw std::runtime_error("--list must be used alone");
    if (options.list) return options;
    if (options.synthetic) {
        if (!options.left.empty() || !options.right.empty() || !options.trigger.empty() || options.capture.device_clock_hz ||
            !options.left_profile.empty() || !options.right_profile.empty()) {
            throw std::runtime_error("--synthetic cannot take MVS camera settings");
        }
    } else if (options.left.empty() || options.right.empty() || options.left == options.right) {
        throw std::runtime_error("provide two distinct camera serials or use --synthetic");
    }
    if (options.calibration.empty() == options.snapshot.empty()) {
        throw std::runtime_error("provide exactly one of --snapshot or --calibration");
    }
    if (!options.snapshot.empty() && (!options.output.empty() || options.frames != 0 || options.require_bound_calibration)) {
        throw std::runtime_error("--snapshot cannot take --output, --frames or --require-bound-calibration");
    }
    return options;
}

class SyntheticCamera final : public Camera {
public:
    SyntheticCamera(bool right, int fps, Clock::time_point start)
        : right_(right), fps_(fps), start_(start) {}

    std::optional<Frame> read() override {
        const auto timestamp = start_ + std::chrono::nanoseconds(index_ * 1000000000LL / fps_);
        std::this_thread::sleep_until(timestamp);
        Image image{640, 360, std::vector<std::uint8_t>(640 * 360 * 3)};
        const auto moving_x = static_cast<int>((index_ * 9) % 1120);
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                const int world_x = x + (right_ ? 512 : 0);
                const int world_y = y - (right_ ? 8 : 0);
                auto* pixel = image.bgr.data() + (y * image.width + x) * 3;
                const bool grid = world_x % 80 < 2 || (world_y >= 0 && world_y % 40 < 2);
                const bool object = world_x >= moving_x && world_x < moving_x + 32 && world_y > 115 && world_y < 235;
                pixel[0] = grid ? 220 : (object ? 30 : 50 + world_x % 143);
                pixel[1] = grid ? 220 : (object ? 45 : 70 + (world_y + 8) % 112);
                pixel[2] = grid ? 220 : (object ? 245 : 55 + world_x / 8);
            }
        }
        ++index_;
        return Frame{std::move(image), timestamp, 0, static_cast<std::uint32_t>(index_), 0};
    }

private:
    bool right_;
    int fps_;
    Clock::time_point start_;
    long long index_ = 0;
};

class StitchStream final : public lystream::StreamControl {
public:
    StitchStream(PairStream& source, Calibration calibration, RawOutput& output, int threads)
        : StreamControl(&source), stitcher_(std::move(calibration), threads), output_(output) {}

private:
    void DataTransferCall(void* data) override {
        const auto& pair = *static_cast<const Pair*>(data);
        const auto& image = stitcher_.stitch(pair.left.image, pair.right.image);
        output_.write(image);
    }

    Stitcher stitcher_;
    RawOutput& output_;
};

void save_ppm(std::filesystem::path path, const Image& image) {
    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << image.width << ' ' << image.height << "\n255\n";
    auto rgb = image.bgr;
    for (std::size_t i = 0; i < rgb.size(); i += 3) std::swap(rgb[i], rgb[i + 2]);
    out.write(reinterpret_cast<const char*>(rgb.data()), rgb.size()).flush();
    if (!out) throw std::runtime_error("could not save " + path.string());
}

void run(const Options& options) {
#ifdef ENABLE_MVS
    std::optional<MvsRuntime> mvs;
    if (!options.synthetic) mvs.emplace();
    if (options.list) {
        list_mvs_cameras();
        return;
    }
#else
    if (!options.synthetic) throw std::runtime_error("build with -DENABLE_MVS=ON to use real cameras");
#endif

    std::array<PairStream::CameraFactory, 2> cameras;
    if (options.synthetic) {
        const auto start = Clock::now() + 150ms;
        cameras[0] = [fps = options.fps, start] { return std::make_unique<SyntheticCamera>(false, fps, start); };
        cameras[1] = [fps = options.fps, start] { return std::make_unique<SyntheticCamera>(true, fps, start); };
    }
#ifdef ENABLE_MVS
    else {
        cameras[0] = [&] {
            return open_mvs_camera(options.left, options.trigger, options.capture.device_clock_hz, options.left_profile);
        };
        cameras[1] = [&] {
            return open_mvs_camera(options.right, options.trigger, options.capture.device_clock_hz, options.right_profile);
        };
    }
#endif

    PairStream source(std::move(cameras), options.capture);
    const auto stopped = [] { return interrupted != 0; };
    if (!options.snapshot.empty()) {
        source.DataTransfer([&](void* data) {
            const auto& pair = *static_cast<const Pair*>(data);
            save_ppm(std::filesystem::path(options.snapshot).concat("-left.ppm"), pair.left.image);
            save_ppm(std::filesystem::path(options.snapshot).concat("-right.ppm"), pair.right.image);
        });
        source.StartStream();
        source.run(1, stopped);
        source.StopStream();
        return;
    }

    auto calibration = load_calibration(options.calibration);
    if (options.require_bound_calibration && !calibration.binding) {
        throw std::runtime_error("calibration must include the camera serials and dimensions");
    }
    if (calibration.binding && (calibration.binding->left_serial != options.left ||
                                calibration.binding->right_serial != options.right)) {
        throw std::runtime_error("calibration camera serials do not match the selected cameras");
    }
    if (!options.output.empty() && std::filesystem::exists(options.output) &&
        std::filesystem::equivalent(options.calibration, options.output)) {
        throw std::runtime_error("video output cannot overwrite the calibration file");
    }
    RawOutput output(options.output, options.write_timeout);
    std::cerr << "output=bgr24 size=" << calibration.width << 'x' << calibration.height << '\n';
    StitchStream stitch(source, std::move(calibration), output, options.stitch_threads);
    stitch.StartStream();
    source.run(options.frames, stopped);
    stitch.StopStream();
    output.finish();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && std::string_view(argv[1]) == "--help")) {
        usage(std::cout);
        return 0;
    }
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
#ifdef SIGPIPE
    std::signal(SIGPIPE, SIG_IGN);
#endif
    try {
        run(parse(argc, argv));
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "dual-hik-stitch: " << error.what() << '\n';
        return 1;
    }
}
