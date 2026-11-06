#pragma once

#include "camera.hpp"

#include <array>
#include <filesystem>

struct Calibration {
    struct Binding {
        std::string left_serial, right_serial;
        std::array<int, 4> sizes{};
    };

    int width;
    int height;
    int left_x;
    int left_y;
    std::array<double, 8> right_from_canvas;
    std::array<int, 4> minimum_input_sizes{};
    std::optional<Binding> binding;
};

Calibration load_calibration(const std::filesystem::path& path);

struct StitchOptions {
    int threads = 1;
    bool auto_exposure = false;
    bool auto_color = false;
    bool grayscale = false;
};

class Stitcher {
public:
    explicit Stitcher(Calibration calibration, StitchOptions options = {});
    const Image& stitch(const Image& left, const Image& right);

private:
    struct Pixel {
        float right_x = -1;
        float right_y = -1;
        float right_weight = 0;
    };

    void prepare(const Image& left, const Image& right);
    void update_correction(const Image& left, const Image& right);

    Calibration calibration_;
    StitchOptions options_;
    std::array<int, 4> source_sizes_{};
    std::vector<Pixel> map_;
    std::vector<std::size_t> samples_;
    float exposure_gain_ = 1;
    std::array<float, 3> color_gains_{1, 1, 1};
    bool exposure_initialized_ = false, color_initialized_ = false;
    unsigned int frames_until_update_ = 0;
    Image result_;
};
