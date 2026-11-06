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

Stitcher::Stitcher(Calibration calibration, int threads)
    : calibration_(std::move(calibration)), threads_(threads) {
#ifndef _OPENMP
    if (threads_ != 1) throw std::runtime_error("parallel stitching requires a build with OpenMP");
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
    source_sizes_ = sizes;
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
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads_) if(threads_ > 1)
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
                continue;
            }

            const int rx = static_cast<int>(pixel.right_x);
            const int ry = static_cast<int>(pixel.right_y);
            const float dx = pixel.right_x - rx;
            const float dy = pixel.right_y - ry;
            const int next_x = std::min(rx + 1, right.width - 1);
            const int next_y = std::min(ry + 1, right.height - 1);
            const auto a = offset(rx, ry, right);
            const auto b = offset(next_x, ry, right);
            const auto c = offset(rx, next_y, right);
            const auto d = offset(next_x, next_y, right);
            for (int ch = 0; ch < 3; ++ch) {
                const float upper = (1 - dx) * right.bgr[a + ch] + dx * right.bgr[b + ch];
                const float lower = (1 - dx) * right.bgr[c + ch] + dx * right.bgr[d + ch];
                float value = (1 - dy) * upper + dy * lower;
                if (pixel.right_weight < 1) {
                    value = pixel.right_weight * value + (1 - pixel.right_weight) * left.bgr[offset(lx, ly, left) + ch];
                }
                result_.bgr[target + ch] = static_cast<std::uint8_t>(value + 0.5f);
            }
        }
    }
    return result_;
}
