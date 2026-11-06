#include "pipeline.hpp"
#include "raw_output.hpp"

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
#ifndef _WIN32
        output_deadline();
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
