#include "camera.hpp"
#include "stitch.hpp"
#include "stream_control.h"

#include <atomic>
#include <charconv>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

using namespace std::chrono_literals;
volatile std::sig_atomic_t interrupted = 0;

void interrupt(int) { interrupted = 1; }

struct Options {
    bool synthetic = false;
    bool list = false;
    std::string left, right, trigger;
    std::filesystem::path calibration, snapshot, output;
    int fps = 30;
    int frames = 0;
    int max_skew_ms = 15;
};

void usage(std::ostream& out) {
    out << "dual-hik-stitch --list\n"
           "dual-hik-stitch --left SERIAL --right SERIAL --snapshot PREFIX\n"
           "dual-hik-stitch (--synthetic | --left SERIAL --right SERIAL) --calibration FILE\n"
           "  [--frames N] [--fps N (synthetic source)] [--max-skew-ms N]\n"
           "  [--trigger Line0] [--output FILE (default: raw BGR24 on stdout)]\n";
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
        else if (flag == "--left") options.left = value();
        else if (flag == "--right") options.right = value();
        else if (flag == "--trigger") options.trigger = value();
        else if (flag == "--calibration") options.calibration = value();
        else if (flag == "--snapshot") options.snapshot = value();
        else if (flag == "--output") options.output = value();
        else if (flag == "--fps") options.fps = number(value(), 1, 240);
        else if (flag == "--frames") options.frames = number(value(), 1, 10000000);
        else if (flag == "--max-skew-ms") options.max_skew_ms = number(value(), 1, 1000);
        else throw std::runtime_error("unknown option: " + std::string(flag));
    }
    if (options.list && argc != 2) throw std::runtime_error("--list must be used alone");
    if (options.list) return options;
    if (options.synthetic) {
        if (!options.left.empty() || !options.right.empty() || !options.trigger.empty()) {
            throw std::runtime_error("--synthetic cannot take camera serials or trigger settings");
        }
    } else if (options.left.empty() || options.right.empty() || options.left == options.right) {
        throw std::runtime_error("provide two distinct camera serials or use --synthetic");
    }
    if (options.calibration.empty() == options.snapshot.empty()) {
        throw std::runtime_error("provide exactly one of --snapshot or --calibration");
    }
    if (!options.snapshot.empty() && (!options.output.empty() || options.frames != 0)) {
        throw std::runtime_error("--snapshot cannot take --output or --frames");
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
        return Frame{std::move(image), timestamp};
    }

private:
    bool right_;
    int fps_;
    Clock::time_point start_;
    long long index_ = 0;
};

struct Pair {
    Frame left, right;
};

class PairStream final : public lystream::Stream {
public:
    PairStream(std::unique_ptr<Camera> left, std::unique_ptr<Camera> right, int max_skew_ms)
        : cameras_{std::move(left), std::move(right)}, max_skew_(max_skew_ms) {}

    void StartStream() override { isrun_ = ++stream_para_.ref_count > 0; }
    void StopStream() override {
        if (stream_para_.ref_count > 0) isrun_ = --stream_para_.ref_count > 0;
    }

    void run(int limit) {
        std::thread left, right;
        const auto join = [&] {
            stop_capture_ = true;
            if (left.joinable()) left.join();
            if (right.joinable()) right.join();
        };
        try {
            left = std::thread([this] { capture(0); });
            right = std::thread([this] { capture(1); });
            auto last_pair = Clock::now();
            while (!interrupted && isrun_ && (limit == 0 || emitted_ < limit)) {
                if (auto pair = next_pair()) {
                    last_pair = Clock::now();
                    for (const auto& output : ondata_list_) output(&*pair);
                    ++emitted_;
                } else if (Clock::now() - last_pair > 10s) {
                    throw std::runtime_error("no frame pair for 10 seconds; check camera streams, trigger and skew");
                }
            }
        } catch (...) {
            join();
            throw;
        }
        join();
        std::cerr << "paired=" << emitted_ << " skew_dropped=" << skew_dropped_
                  << " queue_dropped=" << queue_dropped_ << '\n';
    }

private:
    void capture(std::size_t side) {
        try {
            while (!stop_capture_) {
                auto frame = cameras_[side]->read();
                if (!frame || stop_capture_) continue;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    auto& queue = queues_[side];
                    if (queue.size() >= 4) {
                        queue.pop_front();
                        ++queue_dropped_;
                    }
                    queue.push_back(std::move(*frame));
                }
                ready_.notify_one();
            }
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!error_) error_ = std::current_exception();
            }
            ready_.notify_one();
        }
    }

    std::optional<Pair> next_pair() {
        std::unique_lock<std::mutex> lock(mutex_);
        auto& left = queues_[0];
        auto& right = queues_[1];
        ready_.wait_for(lock, 100ms, [&] { return error_ || (!left.empty() && !right.empty()); });
        if (error_) std::rethrow_exception(error_);
        while (!left.empty() && !right.empty()) {
            const auto difference = left.front().received - right.front().received;
            if (difference > max_skew_) right.pop_front();
            else if (difference < -max_skew_) left.pop_front();
            else {
                Pair pair{std::move(left.front()), std::move(right.front())};
                left.pop_front();
                right.pop_front();
                return pair;
            }
            ++skew_dropped_;
        }
        return std::nullopt;
    }

    std::unique_ptr<Camera> cameras_[2];
    std::chrono::milliseconds max_skew_;
    std::atomic<bool> stop_capture_{false};
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Frame> queues_[2];
    std::exception_ptr error_;
    int emitted_ = 0;
    std::uint64_t skew_dropped_ = 0;
    std::uint64_t queue_dropped_ = 0;
};

class StitchStream final : public lystream::StreamControl {
public:
    StitchStream(PairStream& source, Calibration calibration, std::ostream& output)
        : StreamControl(&source), stitcher_(calibration), output_(output) {}

private:
    void DataTransferCall(void* data) override {
        const auto& pair = *static_cast<const Pair*>(data);
        const auto& image = stitcher_.stitch(pair.left.image, pair.right.image);
        if (!output_.write(reinterpret_cast<const char*>(image.bgr.data()), image.bgr.size())) {
            throw std::runtime_error("could not write the stitched video frame");
        }
    }

    Stitcher stitcher_;
    std::ostream& output_;
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

    std::unique_ptr<Camera> left, right;
    if (options.synthetic) {
        const auto start = Clock::now() + 150ms;
        left = std::make_unique<SyntheticCamera>(false, options.fps, start);
        right = std::make_unique<SyntheticCamera>(true, options.fps, start);
    }
#ifdef ENABLE_MVS
    else {
        left = open_mvs_camera(options.left, options.trigger);
        right = open_mvs_camera(options.right, options.trigger);
    }
#endif

    PairStream source(std::move(left), std::move(right), options.max_skew_ms);
    if (!options.snapshot.empty()) {
        source.DataTransfer([&](void* data) {
            const auto& pair = *static_cast<const Pair*>(data);
            save_ppm(std::filesystem::path(options.snapshot).concat("-left.ppm"), pair.left.image);
            save_ppm(std::filesystem::path(options.snapshot).concat("-right.ppm"), pair.right.image);
        });
        source.StartStream();
        source.run(1);
        source.StopStream();
        return;
    }

    const auto calibration = load_calibration(options.calibration);
    std::ofstream file;
    if (!options.output.empty()) {
        file.open(options.output, std::ios::binary);
        if (!file) throw std::runtime_error("could not open output file");
    }
#ifdef _WIN32
    else if (_setmode(_fileno(stdout), _O_BINARY) < 0) {
        throw std::runtime_error("could not set stdout binary mode");
    }
#endif
    auto& output = options.output.empty() ? std::cout : static_cast<std::ostream&>(file);
    std::cerr << "output=bgr24 size=" << calibration.width << 'x' << calibration.height << '\n';
    StitchStream stitch(source, calibration, output);
    stitch.StartStream();
    source.run(options.frames);
    stitch.StopStream();
    if (!output.flush()) throw std::runtime_error("could not flush the stitched video");
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
