#pragma once

#include "camera.hpp"
#include "stream.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>

struct Pair {
    Frame left;
    Frame right;
};

struct CapturePolicy {
    std::chrono::microseconds max_skew{15000};
    std::chrono::milliseconds no_pair_timeout{10000};
    std::chrono::milliseconds camera_timeout{2000};
    std::chrono::milliseconds report_interval{5000};
    std::uint64_t device_clock_hz = 0;
};

class PairStream final : public lystream::Stream {
public:
    using CameraFactory = std::function<std::unique_ptr<Camera>()>;

    PairStream(std::array<CameraFactory, 2> factories, CapturePolicy policy);
    void StartStream() override;
    void StopStream() override;
    void run(std::uint64_t limit, const std::function<bool()>& stop_requested);

private:
    struct Input {
        std::deque<Frame> queue;
        std::uint64_t received = 0, damaged = 0, missing = 0, queue_dropped = 0;
        std::uint64_t skew_dropped = 0, reconnects = 0;
        bool online = false;
    };

    void capture(std::size_t side);
    std::optional<Pair> next_pair();
    void report() const;
    std::chrono::microseconds::rep difference_us(const Frame& left, const Frame& right) const;

    std::array<CameraFactory, 2> factories_;
    CapturePolicy policy_;
    std::atomic<bool> stopping_{false};
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::array<Input, 2> inputs_;
    std::uint64_t epoch_ = 0;
    std::exception_ptr error_;
    Clock::time_point started_{};
    std::uint64_t emitted_ = 0;
    std::chrono::microseconds::rep max_skew_us_ = 0, max_handler_us_ = 0;
};
