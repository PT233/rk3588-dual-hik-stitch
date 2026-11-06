#pragma once

#include "camera.hpp"

#include <array>
#include <filesystem>

struct Calibration {
    int width;
    int height;
    int left_x;
    int left_y;
    std::array<double, 8> right_from_canvas;
};

Calibration load_calibration(const std::filesystem::path& path);

class Stitcher {
public:
    explicit Stitcher(Calibration calibration);
    const Image& stitch(const Image& left, const Image& right);

private:
    struct Pixel {
        float right_x = -1;
        float right_y = -1;
        float right_weight = 0;
    };

    void prepare(const Image& left, const Image& right);

    Calibration calibration_;
    std::array<int, 4> source_sizes_{};
    std::vector<Pixel> map_;
    Image result_;
};
