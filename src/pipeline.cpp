#include "pipeline.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

namespace {
using namespace std::chrono_literals;
using Microseconds = std::chrono::microseconds;
constexpr const char* side_name(std::size_t side) { return side == 0 ? "left" : "right"; }
}

PairStream::PairStream(std::array<CameraFactory, 2> factories, CapturePolicy policy)
    : factories_(std::move(factories)), policy_(policy) {}

void PairStream::StartStream() { isrun_ = ++stream_para_.ref_count > 0; }

void PairStream::StopStream() {
    if (stream_para_.ref_count > 0) isrun_ = --stream_para_.ref_count > 0;
}

void PairStream::run(std::uint64_t limit, const std::function<bool()>& stop_requested) {
    if (!isrun_ || stopping_) throw std::logic_error("PairStream must be started and can run only once");
    started_ = Clock::now();
    std::array<std::thread, 2> workers;
    const auto check_deadline = [&] {
        if (policy_.run_timeout.count() > 0 && Clock::now() - started_ >= policy_.run_timeout) {
            throw std::runtime_error("capture run deadline exceeded");
        }
    };
    const auto join = [&] {
        stopping_ = true;
        ready_.notify_all();
        for (auto& worker : workers) {
            if (worker.joinable()) worker.join();
        }
    };
    try {
        workers[0] = std::thread([this] { capture(0); });
        workers[1] = std::thread([this] { capture(1); });
        auto last_pair = Clock::now();
        auto next_report = last_pair + policy_.report_interval;
        while (!stop_requested() && isrun_ && (limit == 0 || emitted_ < limit)) {
            if (auto pair = next_pair()) {
                check_deadline();
                last_pair = Clock::now();
                for (const auto& output : ondata_list_) output(&*pair);
                ++emitted_;
                const auto elapsed = std::chrono::duration_cast<Microseconds>(Clock::now() - last_pair);
                max_handler_us_ = std::max(max_handler_us_, elapsed.count());
            } else if (!stop_requested() && Clock::now() - last_pair >= policy_.no_pair_timeout) {
                throw std::runtime_error("frame pair timeout; check camera status and timestamp configuration");
            }
            check_deadline();
            if (Clock::now() >= next_report) {
                report();
                next_report = Clock::now() + policy_.report_interval;
            }
        }
    } catch (...) {
        join();
        report();
        throw;
    }
    join();
    report();
}

void PairStream::capture(std::size_t side) {
    try {
        auto backoff = 250ms;
        while (!stopping_) {
            try {
                auto camera = factories_[side]();
                if (!camera) throw std::logic_error("camera factory returned null");
                auto last_frame = Clock::now();
                std::optional<std::uint32_t> last_sequence;
                std::uint64_t last_ticks = 0;
                while (!stopping_) {
                    std::uint64_t generation;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        generation = epoch_;
                    }
                    auto frame = camera->read();
                    if (!frame) {
                        if (Clock::now() - last_frame >= policy_.camera_timeout) {
                            throw CameraUnavailable("no frame received within camera timeout");
                        }
                        continue;
                    }
                    last_frame = Clock::now();
                    if (policy_.device_clock_hz && !frame->lost_packets &&
                        (frame->device_ticks == 0 || frame->device_ticks <= last_ticks)) {
                        throw CameraUnavailable("missing or non-increasing device timestamp");
                    }
                    if (!frame->lost_packets) last_ticks = frame->device_ticks;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        auto& input = inputs_[side];
                        ++input.received;
                        if (frame->sequence && last_sequence) {
                            const std::uint32_t delta = *frame->sequence - *last_sequence;
                            if (delta > 1 && delta < 0x80000000u) input.missing += delta - 1;
                        }
                        last_sequence = frame->sequence;
                        if (frame->lost_packets) {
                            ++input.damaged;
                            continue;
                        }
                        if (stopping_ || generation != epoch_) continue;
                        if (!input.online) {
                            std::cerr << "camera=" << side_name(side) << " state=streaming\n";
                            input.online = true;
                        }
                        backoff = 250ms;
                        if (input.queue.size() == 4) {
                            input.queue.pop_front();
                            ++input.queue_dropped;
                        }
                        input.queue.push_back(std::move(*frame));
                    }
                    ready_.notify_all();
                }
            } catch (const CameraUnavailable& error) {
                std::unique_lock<std::mutex> lock(mutex_);
                auto& input = inputs_[side];
                input.online = false;
                ++input.reconnects;
                ++epoch_;
                for (auto& source : inputs_) source.queue.clear();
                std::cerr << "camera=" << side_name(side) << " state=reconnecting retry_ms="
                          << backoff.count() << " reason=" << error.what() << '\n';
                ready_.wait_for(lock, backoff, [&] { return stopping_.load(); });
                backoff = std::min(backoff * 2, 4000ms);
            }
        }
    } catch (...) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!error_) error_ = std::current_exception();
        }
        ready_.notify_all();
    }
}

Microseconds::rep PairStream::difference_us(const Frame& left, const Frame& right) const {
    if (!policy_.device_clock_hz) {
        return std::chrono::duration_cast<Microseconds>(left.received - right.received).count();
    }
    const auto older = std::min(left.device_ticks, right.device_ticks);
    const auto newer = std::max(left.device_ticks, right.device_ticks);
    const auto us = static_cast<long double>(newer - older) * 1000000 / policy_.device_clock_hz;
    const auto value = us >= std::numeric_limits<Microseconds::rep>::max()
        ? std::numeric_limits<Microseconds::rep>::max() : static_cast<Microseconds::rep>(us);
    return left.device_ticks < right.device_ticks ? -value : value;
}

std::optional<Pair> PairStream::next_pair() {
    std::unique_lock<std::mutex> lock(mutex_);
    auto& left = inputs_[0].queue;
    auto& right = inputs_[1].queue;
    ready_.wait_for(lock, 100ms, [&] { return error_ || (!left.empty() && !right.empty()); });
    if (error_) std::rethrow_exception(error_);
    const auto max_skew = policy_.max_skew.count();
    while (!left.empty() && !right.empty()) {
        const auto difference = difference_us(left.front(), right.front());
        if (difference > max_skew) {
            right.pop_front();
            ++inputs_[1].skew_dropped;
        } else if (difference < -max_skew) {
            left.pop_front();
            ++inputs_[0].skew_dropped;
        } else {
            max_skew_us_ = std::max(max_skew_us_, difference < 0 ? -difference : difference);
            Pair pair{std::move(left.front()), std::move(right.front()), epoch_, difference};
            left.pop_front();
            right.pop_front();
            return pair;
        }
    }
    return std::nullopt;
}

void PairStream::report() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream line;
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - started_);
    line << "paired=" << emitted_ << " elapsed_ms=" << elapsed.count()
         << " sync=" << (policy_.device_clock_hz ? "device" : "host")
         << " pair_skew_max_us=" << max_skew_us_ << " handler_max_us=" << max_handler_us_;
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        const auto& source = inputs_[i];
        const auto* prefix = side_name(i);
        line << ' ' << prefix << "_online=" << source.online << ' ' << prefix << "_received=" << source.received
             << ' ' << prefix << "_damaged=" << source.damaged
             << ' ' << prefix << "_sequence_missing=" << source.missing << ' ' << prefix << "_queue_dropped=" << source.queue_dropped
             << ' ' << prefix << "_skew_dropped=" << source.skew_dropped << ' ' << prefix << "_reconnects=" << source.reconnects;
    }
    std::cerr << line.str() << '\n';
}
