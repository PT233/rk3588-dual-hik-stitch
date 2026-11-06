#include "raw_output.hpp"

#include <algorithm>
#include <cerrno>
#include <system_error>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <iostream>
#else
#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#ifdef _WIN32

RawOutput::RawOutput(const std::filesystem::path& path, std::chrono::milliseconds) {
    if (path.empty()) {
        if (_isatty(_fileno(stdout))) throw std::runtime_error("redirect raw video stdout or use --output");
        if (_setmode(_fileno(stdout), _O_BINARY) < 0) throw std::runtime_error("could not set stdout binary mode");
        output_ = &std::cout;
    } else {
        file_.open(path, std::ios::binary);
        if (!file_) throw std::runtime_error("could not open output file");
        output_ = &file_;
    }
}

RawOutput::~RawOutput() = default;

void RawOutput::write(const Image& image) {
    if (!output_->write(reinterpret_cast<const char*>(image.bgr.data()), image.bgr.size())) {
        throw std::runtime_error("could not write video frame");
    }
}

void RawOutput::finish() {
    if (!output_->flush()) throw std::runtime_error("could not flush video output");
    if (file_.is_open()) {
        file_.close();
        if (!file_) throw std::runtime_error("could not close output file");
    }
}

#else

namespace {
[[noreturn]] void fail(const char* action) {
    throw std::system_error(errno, std::generic_category(), action);
}
}

RawOutput::RawOutput(const std::filesystem::path& path, std::chrono::milliseconds timeout)
    : timeout_(timeout) {
    if (!path.empty()) {
        fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NONBLOCK, 0666);
        if (fd_ < 0) fail("open video output");
        owns_fd_ = true;
        return;
    }
    fd_ = STDOUT_FILENO;
    if (::isatty(fd_)) throw std::runtime_error("redirect raw video stdout or use --output");
    original_flags_ = ::fcntl(fd_, F_GETFL);
    if (original_flags_ < 0) fail("read video output flags");
    if (::fcntl(fd_, F_SETFL, original_flags_ | O_NONBLOCK) < 0) fail("set non-blocking video output");
}

RawOutput::~RawOutput() {
    if (owns_fd_) ::close(fd_);
    else if (original_flags_ >= 0) ::fcntl(fd_, F_SETFL, original_flags_);
}

void RawOutput::write(const Image& image) {
    const auto deadline = Clock::now() + timeout_;
    std::size_t offset = 0;
    while (offset < image.bgr.size()) {
        const auto remaining = deadline - Clock::now();
        if (remaining <= Clock::duration::zero()) throw std::runtime_error("video output deadline exceeded");
        const auto written = ::write(fd_, image.bgr.data() + offset, image.bgr.size() - offset);
        if (written > 0) {
            offset += static_cast<std::size_t>(written);
            continue;
        }
        if (written == 0) throw std::runtime_error("video output wrote zero bytes");
        if (errno == EINTR) continue;
        if (errno != EAGAIN && errno != EWOULDBLOCK) fail("write video output");

        pollfd ready{fd_, POLLOUT, 0};
        const auto wait_ms = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
        const int result = ::poll(&ready, 1, static_cast<int>(wait_ms));
        if (result == 0) throw std::runtime_error("video output deadline exceeded");
        if (result < 0 && errno != EINTR) fail("poll video output");
        if (result > 0 && (ready.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            throw std::runtime_error("video output consumer disconnected");
        }
    }
}

void RawOutput::finish() {
    if (!owns_fd_) return;
    owns_fd_ = false;
    if (::close(fd_) < 0) fail("close video output");
}

#endif
