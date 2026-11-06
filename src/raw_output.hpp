#pragma once

#include "camera.hpp"

#include <filesystem>
#ifdef _WIN32
#include <fstream>
#endif

class RawOutput {
public:
    RawOutput(const std::filesystem::path& path, std::chrono::milliseconds timeout);
    ~RawOutput();
    RawOutput(const RawOutput&) = delete;
    RawOutput& operator=(const RawOutput&) = delete;
    void write(const Image& image);
    void finish();

private:
#ifdef _WIN32
    std::ofstream file_;
    std::ostream* output_ = nullptr;
#else
    int fd_ = -1;
    int original_flags_ = -1;
    bool owns_fd_ = false;
    std::chrono::milliseconds timeout_;
#endif
};
