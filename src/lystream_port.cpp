#include <string>
#include "stream_control.h"

#include <stdexcept>

namespace lystream {

Stream::Stream() { ondata_list_.reserve(max_num_); }
Stream::~Stream() = default;

void Stream::setName(const std::string& name) { stream_para_.name = name; }
bool Stream::getStreamStatus() { return isrun_; }
Stream::StreamPara Stream::getStreamPara() { return stream_para_; }

void Stream::DataTransfer(const std::function<void(void*)>& ondata) {
    ondata_list_.push_back(ondata);
}

StreamControl::StreamControl(Stream* stream, PolicyMode mode, unsigned int fps)
    : execute_mode_(mode), stream_(stream) {
    if (!stream || (mode == PolicyMode::ExecuteInterval && (fps == 0 || fps > 1000000))) {
        throw std::invalid_argument("invalid lystream input or frame rate");
    }
    if (mode == PolicyMode::ExecuteInterval) {
        time_tick_.clock_tick = 1000000 / fps;
        time_tick_.clock_point = std::chrono::steady_clock::now();
    }
}

StreamControl::~StreamControl() = default;

void StreamControl::StartStream() {
    if (first_run_) {
        stream_->DataTransfer([this](void* frame) { DataTransferCall(frame); });
        first_run_ = false;
    }
    stream_->StartStream();
    ++stream_para_.ref_count;
    stream_para_.size = stream_->getStreamPara().size;
    isrun_ = true;
}

void StreamControl::StopStream() {
    if (stream_para_.ref_count == 0) return;
    stream_->StopStream();
    isrun_ = --stream_para_.ref_count > 0;
}

}  // namespace lystream
