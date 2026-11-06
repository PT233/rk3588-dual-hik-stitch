#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;

struct Image {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> bgr;
};

struct Frame {
    Image image;
    Clock::time_point received;
};

class Camera {
public:
    virtual ~Camera() = default;
    virtual std::optional<Frame> read() = 0;
};

#ifdef ENABLE_MVS
class MvsRuntime {
public:
    MvsRuntime();
    ~MvsRuntime();
    MvsRuntime(const MvsRuntime&) = delete;
    MvsRuntime& operator=(const MvsRuntime&) = delete;
};

void list_mvs_cameras();
std::unique_ptr<Camera> open_mvs_camera(const std::string& serial, const std::string& trigger);
#endif
