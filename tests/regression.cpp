#include "pipeline.hpp"
#include "raw_output.hpp"
#include "stitch.hpp"

#include <iostream>
#include <thread>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace {
using namespace std::chrono_literals;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct Event {
    int after_ms;
    std::uint64_t ticks;
    std::uint32_t lost_packets = 0;
};

class Replay final : public Camera {
public:
    Replay(Clock::time_point start, std::vector<Event> events)
        : start_(start), events_(std::move(events)) {}

    std::optional<Frame> read() override {
        if (index_ == events_.size()) {
            std::this_thread::sleep_for(20ms);
            return std::nullopt;
        }
        const auto event = events_[index_++];
        std::this_thread::sleep_until(start_ + std::chrono::milliseconds(event.after_ms));
        if (!event.ticks) throw CameraUnavailable("injected disconnect");
        return Frame{{}, Clock::now(), event.ticks, static_cast<std::uint32_t>(index_), event.lost_packets};
    }

private:
    Clock::time_point start_;
    std::vector<Event> events_;
    std::size_t index_ = 0;
};

void reconnect_and_resynchronize() {
    const auto start = Clock::now();
    int opens = 0;
    CapturePolicy policy;
    policy.max_skew = 100us;
    policy.device_clock_hz = 1000000;
    policy.no_pair_timeout = 2s;
    PairStream source({
        [&] {
            if (++opens == 1) {
                return std::make_unique<Replay>(start, std::vector<Event>{{30, 1000}, {80, 2000, 2}, {120, 3000}, {210, 0}});
            }
            return std::make_unique<Replay>(start, std::vector<Event>{{650, 5000}});
        },
        [&] {
            return std::make_unique<Replay>(start, std::vector<Event>{{55, 1004}, {105, 2004}, {155, 3004}, {400, 4004}, {710, 5004}});
        }
    }, policy);
    std::vector<std::pair<std::uint64_t, std::uint64_t>> pairs;
    source.DataTransfer([&](void* data) {
        const auto& pair = *static_cast<const Pair*>(data);
        pairs.emplace_back(pair.left.device_ticks, pair.right.device_ticks);
    });
    source.StartStream();
    source.run(3, [] { return false; });
    source.StopStream();
    const decltype(pairs) expected{{1000, 1004}, {3000, 3004}, {5000, 5004}};
    require(opens == 2 && pairs == expected, "capture failed to recover and pair by device time");
}

void no_pair_deadline() {
    const auto idle = [] { return std::make_unique<Replay>(Clock::now(), std::vector<Event>{}); };
    CapturePolicy policy;
    policy.no_pair_timeout = 250ms;
    PairStream source({idle, idle}, policy);
    source.StartStream();
    bool timed_out = false;
    try {
        source.run(1, [] { return false; });
    } catch (const std::runtime_error& error) {
        timed_out = std::string(error.what()).find("frame pair timeout") != std::string::npos;
    }
    require(timed_out, "no-pair watchdog failed");
}

Image flat_image(std::array<std::uint8_t, 3> bgr) {
    Image result{48, 32, std::vector<std::uint8_t>(48 * 32 * 3)};
    for (std::size_t i = 0; i < result.bgr.size(); ++i) result.bgr[i] = bgr[i % 3];
    return result;
}

std::array<std::uint8_t, 3> pixel_at(const Image& image, int x, int y) {
    const auto offset = (y * image.width + x) * 3;
    return {image.bgr[offset], image.bgr[offset + 1], image.bgr[offset + 2]};
}

void photometric_matching() {
    using Pixel = std::array<std::uint8_t, 3>;
    Calibration calibration{};
    calibration.width = 72;
    calibration.height = 32;
    calibration.right_from_canvas = {1, 0, -24, 0, 1, 0, 0, 0};

    const auto left = flat_image({72, 112, 144});
    const auto dim = flat_image({48, 75, 96});
    Stitcher reference(calibration);
    require(pixel_at(reference.stitch(left, dim), 60, 10) == Pixel{48, 75, 96}, "default colors changed");

    Stitcher exposure(calibration, StitchOptions{1, true, false, false});
    const auto& bright = exposure.stitch(left, dim);
    require(pixel_at(bright, 60, 10) == Pixel{72, 112, 144}, "exposure gain did not match the reference");
    require(pixel_at(bright, 8, 10) == Pixel{72, 112, 144}, "correction changed the left reference");

    auto tinted = flat_image({54, 84, 96});
    for (int y = 0; y < 7; ++y) {
        for (int x = 0; x < 24; ++x) {
            const auto index = (y * tinted.width + x) * 3;
            tinted.bgr[index] = 220;
            tinted.bgr[index + 1] = 42;
            tinted.bgr[index + 2] = 100;
        }
    }
    Stitcher color(calibration, StitchOptions{1, true, true, false});
    require(pixel_at(color.stitch(left, tinted), 60, 10) == Pixel{72, 112, 144},
            "robust color match failed with an overlap outlier");

    const auto clipped = flat_image({250, 250, 250});
    const auto& invalid = color.stitch(clipped, tinted);
    require(pixel_at(invalid, 60, 10) == Pixel{72, 112, 144}, "invalid samples changed the prior correction");
    for (int i = 0; i < 5; ++i) color.stitch(clipped, tinted);
    require(pixel_at(color.stitch(clipped, tinted), 60, 10) == Pixel{72, 112, 144}, "clipped update was accepted");

    Stitcher gray(calibration, StitchOptions{1, true, true, true});
    const auto& grayscale = gray.stitch(left, tinted);
    require(pixel_at(grayscale, 8, 10) == Pixel{117, 117, 117} &&
            pixel_at(grayscale, 60, 10) == Pixel{117, 117, 117}, "grayscale output does not preserve luminance");
}

#ifndef _WIN32
void output_deadline() {
    int pipe_fds[2];
    require(::pipe(pipe_fds) == 0, "could not create test pipe");
    const int saved_stdout = ::dup(STDOUT_FILENO);
    require(saved_stdout >= 0 && ::dup2(pipe_fds[1], STDOUT_FILENO) >= 0, "could not redirect test stdout");
    ::close(pipe_fds[1]);
    bool timed_out = false;
    try {
        RawOutput out({}, 80ms);
        out.write(Image{1024, 1024, std::vector<std::uint8_t>(1024 * 1024 * 3)});
    } catch (const std::runtime_error& error) {
        timed_out = std::string(error.what()).find("video output deadline exceeded") != std::string::npos;
    }
    ::dup2(saved_stdout, STDOUT_FILENO);
    ::close(saved_stdout);
    ::close(pipe_fds[0]);
    require(timed_out, "full pipe did not time out");
}
#endif
}

int main() {
    try {
        reconnect_and_resynchronize();
        no_pair_deadline();
        photometric_matching();
#ifndef _WIN32
        output_deadline();
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
