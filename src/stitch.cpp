#include "stitch.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace {

using Equation = std::array<double, 9>;

std::array<double, 8> solve(std::array<Equation, 8> system) {
    for (std::size_t col = 0; col < system.size(); ++col) {
        auto pivot = col;
        for (auto row = col + 1; row < system.size(); ++row) {
            if (std::abs(system[row][col]) > std::abs(system[pivot][col])) pivot = row;
        }
        std::swap(system[col], system[pivot]);
        const auto divisor = system[col][col];
        if (std::abs(divisor) < 1e-9) throw std::runtime_error("degenerate calibration points");
        for (auto& coefficient : system[col]) coefficient /= divisor;
        for (std::size_t row = 0; row < system.size(); ++row) {
            if (row == col) continue;
            const auto factor = system[row][col];
            for (std::size_t i = col; i < system[row].size(); ++i) {
                system[row][i] -= factor * system[col][i];
            }
        }
    }

    std::array<double, 8> solution{};
    for (std::size_t i = 0; i < solution.size(); ++i) solution[i] = system[i].back();
    return solution;
}

float edge_distance(float x, float y, const Image& image) {
    return 1 + std::min({x, y, image.width - 1 - x, image.height - 1 - y});
}

bool contains(float x, float y, const Image& image) {
    return x >= 0 && x <= image.width - 1 && y >= 0 && y <= image.height - 1;
}

std::size_t offset(int x, int y, const Image& image) {
    return (static_cast<std::size_t>(y) * image.width + x) * 3;
}

std::array<float, 3> interpolate(float x, float y, const Image& image) {
    const int ix = static_cast<int>(x);
    const int iy = static_cast<int>(y);
    const float dx = x - ix;
    const float dy = y - iy;
    const int next_x = std::min(ix + 1, image.width - 1);
    const int next_y = std::min(iy + 1, image.height - 1);
    const auto a = offset(ix, iy, image), b = offset(next_x, iy, image);
    const auto c = offset(ix, next_y, image), d = offset(next_x, next_y, image);
    std::array<float, 3> value{};
    for (int ch = 0; ch < 3; ++ch) {
        const float upper = (1 - dx) * image.bgr[a + ch] + dx * image.bgr[b + ch];
        const float lower = (1 - dx) * image.bgr[c + ch] + dx * image.bgr[d + ch];
        value[ch] = (1 - dy) * upper + dy * lower;
    }
    return value;
}

template <class Pixel>
float luminance(const Pixel& p) {
    return (29.f * p[0] + 150.f * p[1] + 77.f * p[2]) / 256.f;
}

void grayscale(std::uint8_t* pixel) {
    const auto y = static_cast<std::uint8_t>(luminance(pixel) + 0.5f);
    std::fill_n(pixel, 3, y);
}

float median(std::vector<float>& values) {
    auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

float adapt_gain(float previous, float target, bool initialized) {
    return initialized ? previous + 0.15f * (target - previous) : target;
}

}  // namespace

Calibration load_calibration(const std::filesystem::path& path) {
    std::ifstream input(path);
    Calibration calibration{};
    if (!(input >> calibration.width >> calibration.height >> calibration.left_x >> calibration.left_y) ||
        calibration.width < 2 || calibration.height < 2 || calibration.width > 8192 ||
        calibration.height > 8192 || static_cast<long long>(calibration.width) * calibration.height > 16777216 ||
        std::abs(static_cast<double>(calibration.left_x)) > 8192 ||
        std::abs(static_cast<double>(calibration.left_y)) > 8192) {
        throw std::runtime_error("invalid calibration canvas in " + path.string());
    }

    std::array<Equation, 8> system{};
    for (std::size_t i = 0; i < 4; ++i) {
        double x, y, u, v;
        if (!(input >> x >> y >> u >> v) || !(x >= 0 && x < 8192 && y >= 0 && y < 8192 &&
                                             u >= 0 && u < 8192 && v >= 0 && v < 8192)) {
            throw std::runtime_error("calibration requires four rows of left_x left_y right_x right_y");
        }
        const std::array<double, 4> coordinates{x, y, u, v};
        for (std::size_t j = 0; j < coordinates.size(); ++j) {
            auto& minimum = calibration.minimum_input_sizes[j];
            minimum = std::max(minimum, static_cast<int>(std::ceil(coordinates[j])) + 1);
        }
        x += calibration.left_x;
        y += calibration.left_y;
        system[2 * i] = {x, y, 1, 0, 0, 0, -x * u, -y * u, u};
        system[2 * i + 1] = {0, 0, 0, x, y, 1, -x * v, -y * v, v};
    }
    std::string extra;
    if (input >> extra) {
        Calibration::Binding binding;
        if (extra != "cameras" || !(input >> binding.left_serial >> binding.sizes[0] >> binding.sizes[1]
                                           >> binding.right_serial >> binding.sizes[2] >> binding.sizes[3]) ||
            binding.left_serial == binding.right_serial || (input >> extra)) {
            throw std::runtime_error("expected cameras LEFT_SERIAL WIDTH HEIGHT RIGHT_SERIAL WIDTH HEIGHT");
        }
        for (std::size_t i = 0; i < binding.sizes.size(); ++i) {
            if (binding.sizes[i] < calibration.minimum_input_sizes[i] || binding.sizes[i] > 8192) {
                throw std::runtime_error("calibration points fall outside the bound camera dimensions");
            }
        }
        calibration.binding = std::move(binding);
    }
    calibration.right_from_canvas = solve(system);
    const auto& h = calibration.right_from_canvas;
    for (int x : {0, calibration.width - 1}) {
        for (int y : {0, calibration.height - 1}) {
            if (!(h[6] * x + h[7] * y + 1 > 1e-9)) {
                throw std::runtime_error("calibration horizon intersects the output canvas");
            }
        }
    }
    return calibration;
}

Stitcher::Stitcher(Calibration calibration, StitchOptions options)
    : calibration_(std::move(calibration)), options_(options) {
#ifndef _OPENMP
    if (options_.threads != 1) throw std::runtime_error("parallel stitching requires a build with OpenMP");
#endif
    result_.width = calibration_.width;
    result_.height = calibration_.height;
    result_.bgr.resize(static_cast<std::size_t>(result_.width) * result_.height * 3);
}

void Stitcher::prepare(const Image& left, const Image& right) {
    const std::array<int, 4> sizes{left.width, left.height, right.width, right.height};
    if (calibration_.binding && calibration_.binding->sizes != sizes) {
        throw std::runtime_error("camera dimensions differ from the calibration; check ROI");
    }
    for (std::size_t i = 0; i < sizes.size(); ++i) {
        if (sizes[i] < calibration_.minimum_input_sizes[i]) {
            throw std::runtime_error("calibration points fall outside the captured image");
        }
    }
    const auto& h = calibration_.right_from_canvas;
    map_.assign(result_.bgr.size() / 3, {});
    std::size_t overlap = 0;
    for (int y = 0; y < result_.height; ++y) {
        for (int x = 0; x < result_.width; ++x) {
            const double w = h[6] * x + h[7] * y + 1;
            if (std::abs(w) < 1e-9) continue;
            const float rx = static_cast<float>((h[0] * x + h[1] * y + h[2]) / w);
            const float ry = static_cast<float>((h[3] * x + h[4] * y + h[5]) / w);
            if (!contains(rx, ry, right)) continue;

            auto& pixel = map_[offset(x, y, result_) / 3];
            pixel = {rx, ry, 1};
            const float lx = static_cast<float>(x - calibration_.left_x);
            const float ly = static_cast<float>(y - calibration_.left_y);
            if (contains(lx, ly, left)) {
                const auto weight = edge_distance(rx, ry, right);
                pixel.right_weight = weight / (weight + edge_distance(lx, ly, left));
                ++overlap;
            }
        }
    }
    if (overlap == 0) throw std::runtime_error("the calibrated images do not overlap on the canvas");
    if (options_.auto_exposure || options_.auto_color) {
        const auto stride = (overlap + 1023) / 1024;
        std::size_t seen = 0;
        for (std::size_t i = 0; i < map_.size(); ++i) {
            const auto weight = map_[i].right_weight;
            if (weight > 0 && weight < 1 && seen++ % stride == 0) samples_.push_back(i);
        }
    }
    source_sizes_ = sizes;
}

void Stitcher::update_correction(const Image& left, const Image& right) {
    std::vector<float> exposure;
    std::array<std::vector<float>, 3> colors;
    if (options_.auto_exposure) exposure.reserve(samples_.size());
    if (options_.auto_color) {
        for (auto& channel : colors) channel.reserve(samples_.size());
    }
    for (const auto index : samples_) {
        const int x = static_cast<int>(index % result_.width) - calibration_.left_x;
        const int y = static_cast<int>(index / result_.width) - calibration_.left_y;
        const auto* reference = left.bgr.data() + offset(x, y, left);
        const auto& mapping = map_[index];
        const auto sample = interpolate(mapping.right_x, mapping.right_y, right);
        const auto left_y = luminance(reference), right_y = luminance(sample);
        if (left_y < 16 || right_y < 16 || *std::max_element(reference, reference + 3) >= 245 ||
            *std::max_element(sample.begin(), sample.end()) >= 245) continue;
        const auto brightness_ratio = left_y / right_y;
        if (options_.auto_exposure) exposure.push_back(brightness_ratio);
        if (options_.auto_color && *std::min_element(reference, reference + 3) >= 12 &&
            *std::min_element(sample.begin(), sample.end()) >= 12) {
            for (int ch = 0; ch < 3; ++ch) {
                colors[ch].push_back(reference[ch] / (sample[ch] * brightness_ratio));
            }
        }
    }
    if (exposure.size() >= 32) {
        exposure_gain_ = adapt_gain(exposure_gain_, std::clamp(median(exposure), 0.5f, 2.f), exposure_initialized_);
        exposure_initialized_ = true;
    }
    if (colors[0].size() >= 32) {
        for (int ch = 0; ch < 3; ++ch) {
            const auto target = std::clamp(median(colors[ch]), 0.67f, 1.5f);
            color_gains_[ch] = adapt_gain(color_gains_[ch], target, color_initialized_);
        }
        color_initialized_ = true;
    }
}

const Image& Stitcher::stitch(const Image& left, const Image& right) {
    for (const Image* image : {&left, &right}) {
        if (image->width <= 0 || image->height <= 0 || image->width > 8192 || image->height > 8192 ||
            image->bgr.size() != static_cast<std::size_t>(image->width) * image->height * 3) {
            throw std::runtime_error("invalid BGR frame dimensions or payload");
        }
    }
    if (map_.empty()) {
        prepare(left, right);
    } else if (source_sizes_ != std::array<int, 4>{left.width, left.height, right.width, right.height}) {
        throw std::runtime_error("camera frame size changed; check ROI and recalibrate");
    }
    const bool correct = options_.auto_exposure || options_.auto_color;
    if (correct) {
        if (frames_until_update_ == 0) {
            update_correction(left, right);
            frames_until_update_ = 5;
        }
        --frames_until_update_;
    }
    auto gains = color_gains_;
    for (auto& gain : gains) gain = std::clamp(gain * exposure_gain_, 0.5f, 2.f);
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(options_.threads) if(options_.threads > 1)
#endif
    for (int y = 0; y < result_.height; ++y) {
        for (int x = 0; x < result_.width; ++x) {
            const auto target = offset(x, y, result_);
            const auto& pixel = map_[target / 3];
            const int lx = x - calibration_.left_x;
            const int ly = y - calibration_.left_y;
            if (pixel.right_x < 0) {
                if (contains(lx, ly, left)) {
                    std::copy_n(left.bgr.data() + offset(lx, ly, left), 3, result_.bgr.data() + target);
                } else {
                    std::fill_n(result_.bgr.data() + target, 3, 0);
                }
                if (options_.grayscale) grayscale(result_.bgr.data() + target);
                continue;
            }

            const auto sample = interpolate(pixel.right_x, pixel.right_y, right);
            for (int ch = 0; ch < 3; ++ch) {
                float value = correct ? std::min(sample[ch] * gains[ch], 255.f) : sample[ch];
                if (pixel.right_weight < 1) {
                    value = pixel.right_weight * value + (1 - pixel.right_weight) * left.bgr[offset(lx, ly, left) + ch];
                }
                result_.bgr[target + ch] = static_cast<std::uint8_t>(value + 0.5f);
            }
            if (options_.grayscale) grayscale(result_.bgr.data() + target);
        }
    }
    return result_;
}
