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
    std::filesystem::path calibration, snapshot, output, trace_dir;
    int fps = 30;
    int frames = 0;
    int check_pairs = 0;
    StitchOptions stitch;
    AutoExposure camera_auto_exposure = AutoExposure::profile;
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
           "  [--camera-auto-exposure off|once|continuous] (MVS sensor)\n"
           "  [--auto-exposure] [--auto-color] [--grayscale] (stitched video)\n"
           "  [--check-pairs N (no video)] [--run-timeout-ms N]\n"
           "  [--trace-dir NEW_DIRECTORY (bounded runs only)]\n"
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
        else if (flag == "--auto-exposure") options.stitch.auto_exposure = true;
        else if (flag == "--auto-color") options.stitch.auto_color = true;
        else if (flag == "--grayscale") options.stitch.grayscale = true;
        else if (flag == "--camera-auto-exposure") {
            const auto mode = value();
            if (mode == "off") options.camera_auto_exposure = AutoExposure::off;
            else if (mode == "once") options.camera_auto_exposure = AutoExposure::once;
            else if (mode == "continuous") options.camera_auto_exposure = AutoExposure::continuous;
            else throw std::runtime_error("--camera-auto-exposure requires off, once or continuous");
        }
        else if (flag == "--left") options.left = value();
        else if (flag == "--right") options.right = value();
        else if (flag == "--left-profile") options.left_profile = value();
        else if (flag == "--right-profile") options.right_profile = value();
        else if (flag == "--trigger") options.trigger = value();
        else if (flag == "--calibration") options.calibration = std::filesystem::u8path(value());
        else if (flag == "--snapshot") options.snapshot = std::filesystem::u8path(value());
        else if (flag == "--output") options.output = std::filesystem::u8path(value());
        else if (flag == "--trace-dir") options.trace_dir = std::filesystem::u8path(value());
        else if (flag == "--check-pairs") options.check_pairs = number(value(), 2, 100000);
        else if (flag == "--run-timeout-ms") options.capture.run_timeout = std::chrono::milliseconds(number(value(), 200, 3600000));
        else if (flag == "--fps") options.fps = number(value(), 1, 240);
        else if (flag == "--frames") options.frames = number(value(), 1, 10000000);
        else if (flag == "--max-skew-ms") options.capture.max_skew = std::chrono::milliseconds(number(value(), 1, 1000));
        else if (flag == "--max-skew-us") options.capture.max_skew = std::chrono::microseconds(number(value(), 1, 1000000));
        else if (flag == "--camera-timeout-ms") options.capture.camera_timeout = std::chrono::milliseconds(number(value(), 200, 300000));
        else if (flag == "--pair-timeout-ms") options.capture.no_pair_timeout = std::chrono::milliseconds(number(value(), 200, 300000));
        else if (flag == "--write-timeout-ms") options.write_timeout = std::chrono::milliseconds(number(value(), 10, 30000));
        else if (flag == "--device-clock-hz") options.capture.device_clock_hz = number(value(), 1, 2000000000);
        else if (flag == "--stitch-threads") options.stitch.threads = number(value(), 1, 8);
        else throw std::runtime_error("unknown option: " + std::string(flag));
    }
    if (options.list && argc != 2) throw std::runtime_error("--list must be used alone");
    if (options.list) return options;
    if (options.synthetic) {
        if (!options.left.empty() || !options.right.empty() || !options.trigger.empty() || options.capture.device_clock_hz ||
            !options.left_profile.empty() || !options.right_profile.empty() ||
            options.camera_auto_exposure != AutoExposure::profile) {
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
    if (!options.snapshot.empty() && (options.stitch.auto_exposure || options.stitch.auto_color || options.stitch.grayscale)) {
        throw std::runtime_error("photometric output options require --calibration");
    }
    if (options.check_pairs && (!options.snapshot.empty() || !options.output.empty() || options.frames)) {
        throw std::runtime_error("--check-pairs uses --calibration and cannot take --output or --frames");
    }
    if (!options.trace_dir.empty() && !options.check_pairs && (options.frames == 0 || options.frames > 100000)) {
        throw std::runtime_error("--trace-dir requires --check-pairs or --frames up to 100000");
    }
    if (options.check_pairs && options.capture.run_timeout.count() == 0) options.capture.run_timeout = 10000ms;
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
    StitchStream(PairStream& source, Calibration calibration, RawOutput* output, StitchOptions options,
                 std::ofstream* trace, bool check, bool device_clock)
        : StreamControl(&source), stitcher_(std::move(calibration), options), output_(output),
          trace_(trace), check_(check), device_clock_(device_clock) {}

    std::uint64_t count() const { return count_; }

private:
    static auto micros(Clock::duration duration) {
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }

    void verify_pair(const Pair& pair) {
        if (pair.epoch != 0) throw std::runtime_error("camera reconnected during self-check");
        const Frame* cameras[] = {&pair.left, &pair.right};
        for (std::size_t i = 0; i < 2; ++i) {
            const auto& camera = *cameras[i];
            if (!camera.sequence) throw std::runtime_error("camera sequence is unavailable during self-check");
            if (last_sequence_[i]) {
                const std::uint32_t advance = *camera.sequence - *last_sequence_[i];
                if (advance == 0 || advance >= 0x80000000u || camera.source_pixel_type != pixel_type_[i]) {
                    throw std::runtime_error("camera sequence regressed or pixel format changed during self-check");
                }
            }
            last_sequence_[i] = camera.sequence;
            pixel_type_[i] = camera.source_pixel_type;
        }
    }

    void trace_camera(const Frame& frame, Clock::time_point paired) {
        *trace_ << "{\"sequence\":";
        if (frame.sequence) *trace_ << *frame.sequence;
        else *trace_ << "null";
        *trace_ << ",\"device_ticks\":\"" << frame.device_ticks << "\",\"receive_age_us\":"
                << micros(paired - frame.received) << ",\"width\":" << frame.image.width
                << ",\"height\":" << frame.image.height << ",\"source_pixel_type\":";
        if (frame.source_pixel_type) *trace_ << *frame.source_pixel_type;
        else *trace_ << "null";
        *trace_ << '}';
    }

    void DataTransferCall(void* data) override {
        const auto& pair = *static_cast<const Pair*>(data);
        if (check_) verify_pair(pair);
        const auto paired = Clock::now();
        const auto& image = stitcher_.stitch(pair.left.image, pair.right.image);
        const auto stitched = Clock::now();
        if (output_) output_->write(image);
        const auto written = Clock::now();
        ++count_;
        if (!trace_) return;
        *trace_ << "{\"pair_index\":" << count_ << ",\"epoch\":" << pair.epoch
                << ",\"pair_monotonic_us\":" << micros(paired.time_since_epoch())
                << ",\"pair_skew_us\":" << pair.skew_us << ",\"clock\":\"" << (device_clock_ ? "device" : "host")
                << "\",\"stitch_us\":" << micros(stitched - paired) << ",\"write_us\":";
        if (output_) *trace_ << micros(written - stitched);
        else *trace_ << "null";
        *trace_ << ",\"left\":";
        trace_camera(pair.left, paired);
        *trace_ << ",\"right\":";
        trace_camera(pair.right, paired);
        *trace_ << "}\n";
        if (!*trace_) throw std::runtime_error("could not write frame trace");
    }

    Stitcher stitcher_;
    RawOutput* output_;
    std::ofstream* trace_;
    bool check_, device_clock_;
    std::uint64_t count_ = 0;
    std::array<std::optional<std::uint32_t>, 2> last_sequence_{}, pixel_type_{};
};

void save_ppm(std::filesystem::path path, const Image& image) {
    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << image.width << ' ' << image.height << "\n255\n";
    auto rgb = image.bgr;
    for (std::size_t i = 0; i < rgb.size(); i += 3) std::swap(rgb[i], rgb[i + 2]);
    out.write(reinterpret_cast<const char*>(rgb.data()), rgb.size()).flush();
    if (!out) throw std::runtime_error("could not save " + path.u8string());
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
            return open_mvs_camera(options.left, options.trigger, options.capture.device_clock_hz,
                                   options.left_profile, options.camera_auto_exposure);
        };
        cameras[1] = [&] {
            return open_mvs_camera(options.right, options.trigger, options.capture.device_clock_hz,
                                   options.right_profile, options.camera_auto_exposure);
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
    std::ofstream trace;
    if (!options.trace_dir.empty()) {
        if (!std::filesystem::create_directory(options.trace_dir)) throw std::runtime_error("trace directory already exists");
        const auto trace_file = options.trace_dir / "frames.jsonl";
        trace.open(trace_file, std::ios::binary);
        if (!trace) throw std::runtime_error("could not open frame trace");
        if (!options.output.empty() && std::filesystem::exists(options.output) &&
            std::filesystem::equivalent(trace_file, options.output)) {
            throw std::runtime_error("video output cannot overwrite the frame trace");
        }
    }
    std::optional<RawOutput> output;
    if (!options.check_pairs) output.emplace(options.output, options.write_timeout);
    std::cerr << "output=bgr24 size=" << calibration.width << 'x' << calibration.height << '\n';
    const auto width = calibration.width, height = calibration.height;
    StitchStream stitch(source, std::move(calibration), output ? &*output : nullptr, options.stitch,
                        trace.is_open() ? &trace : nullptr, options.check_pairs != 0, options.capture.device_clock_hz != 0);
    stitch.StartStream();
    source.run(options.check_pairs ? options.check_pairs : options.frames, stopped);
    stitch.StopStream();
    if (output) output->finish();
    if (trace.is_open()) {
        trace.close();
        if (!trace) throw std::runtime_error("could not finalize frame trace");
    }
    if (options.check_pairs) {
        if (stitch.count() != static_cast<std::uint64_t>(options.check_pairs)) throw std::runtime_error("self-check interrupted");
        std::cout << "{\"status\":\"passed\",\"pairs\":" << stitch.count() << ",\"width\":" << width
                  << ",\"height\":" << height << "}\n";
    }
}

int run_cli(int argc, char** argv) {
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

}  // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t** argv) {
    std::vector<std::string> utf8;
    utf8.reserve(argc);
    for (int i = 0; i < argc; ++i) utf8.push_back(std::filesystem::path(argv[i]).u8string());
    std::vector<char*> arguments;
    arguments.reserve(argc);
    for (auto& value : utf8) arguments.push_back(value.data());
    return run_cli(argc, arguments.data());
}
#else
int main(int argc, char** argv) { return run_cli(argc, argv); }
#endif
