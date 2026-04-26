#include "realtime_stab_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace rtsdk {
namespace {

inline int sample_ch(const uint8_t* frame, int w, int h, int c, int channels,
                     int x, int y, int border_mode) {
    if (x >= 0 && x < w && y >= 0 && y < h) {
        return frame[(y * w + x) * channels + c];
    }
    if (border_mode == 0) {
        return 0;
    }
    const int cx = std::min(std::max(x, 0), w - 1);
    const int cy = std::min(std::max(y, 0), h - 1);
    return frame[(cy * w + cx) * channels + c];
}

inline int sad_patch(const uint8_t* prev_gray, const uint8_t* curr_gray,
                     int w, int h, int x, int y, int dx, int dy, int patch_radius) {
    int sad = 0;
    for (int py = -patch_radius; py <= patch_radius; ++py) {
        const int yy0 = y + py;
        const int yy1 = yy0 + dy;
        if (yy0 < 0 || yy0 >= h || yy1 < 0 || yy1 >= h) {
            return std::numeric_limits<int>::max();
        }
        for (int px = -patch_radius; px <= patch_radius; ++px) {
            const int xx0 = x + px;
            const int xx1 = xx0 + dx;
            if (xx0 < 0 || xx0 >= w || xx1 < 0 || xx1 >= w) {
                return std::numeric_limits<int>::max();
            }
            const int a = prev_gray[yy0 * w + xx0];
            const int b = curr_gray[yy1 * w + xx1];
            sad += std::abs(a - b);
        }
    }
    return sad;
}

inline float median(std::vector<float>& vals) {
    if (vals.empty()) return 0.f;
    const size_t mid = vals.size() / 2;
    std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
    float med = vals[mid];
    if ((vals.size() & 1U) == 0U) {
        std::nth_element(vals.begin(), vals.begin() + mid - 1, vals.end());
        med = 0.5f * (med + vals[mid - 1]);
    }
    return med;
}

} // namespace

struct Stabilizer::Impl {
    explicit Impl(const StabilizerConfig& c) : cfg(c) {
        cfg.width = std::max(cfg.width, 16);
        cfg.height = std::max(cfg.height, 16);
        cfg.input_channels = (cfg.input_channels == 1) ? 1 : 3;
        cfg.search_radius = std::max(cfg.search_radius, 1);
        cfg.patch_radius = std::max(cfg.patch_radius, 1);
        cfg.grid_cols = std::max(cfg.grid_cols, 1);
        cfg.grid_rows = std::max(cfg.grid_rows, 1);
        cfg.ema_alpha = std::min(std::max(cfg.ema_alpha, 0.f), 0.9999f);

        gray_prev.resize(cfg.width * cfg.height);
        gray_curr.resize(cfg.width * cfg.height);
        frame_scratch.resize(static_cast<size_t>(cfg.width) * static_cast<size_t>(cfg.height) *
                             static_cast<size_t>(cfg.input_channels));
    }

    StabilizerConfig cfg;
    bool initialized = false;
    std::vector<uint8_t> gray_prev;
    std::vector<uint8_t> gray_curr;
    std::vector<uint8_t> frame_scratch;

    float path_x = 0.f;
    float path_y = 0.f;
    float smooth_x = 0.f;
    float smooth_y = 0.f;

    void to_gray(const uint8_t* input, std::vector<uint8_t>& gray) const {
        const int n = cfg.width * cfg.height;
        if (cfg.input_channels == 1) {
            std::memcpy(gray.data(), input, static_cast<size_t>(n));
            return;
        }

        for (int i = 0; i < n; ++i) {
            const int b = input[i * cfg.input_channels + 0];
            const int g = input[i * cfg.input_channels + 1];
            const int r = input[i * cfg.input_channels + 2];
            gray[i] = static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
        }
    }

    std::pair<float, float> estimate_motion() const {
        std::vector<float> dxs;
        std::vector<float> dys;
        dxs.reserve(static_cast<size_t>(cfg.grid_cols * cfg.grid_rows));
        dys.reserve(static_cast<size_t>(cfg.grid_cols * cfg.grid_rows));

        const int margin = std::max(cfg.patch_radius + cfg.search_radius + 1, 8);
        const int x0 = margin;
        const int y0 = margin;
        const int x1 = cfg.width - margin - 1;
        const int y1 = cfg.height - margin - 1;
        if (x1 <= x0 || y1 <= y0) {
            return std::make_pair(0.f, 0.f);
        }

        for (int gy = 0; gy < cfg.grid_rows; ++gy) {
            const float fy = (cfg.grid_rows == 1) ? 0.f : static_cast<float>(gy) / static_cast<float>(cfg.grid_rows - 1);
            const int y = static_cast<int>(y0 + fy * static_cast<float>(y1 - y0));

            for (int gx = 0; gx < cfg.grid_cols; ++gx) {
                const float fx = (cfg.grid_cols == 1) ? 0.f : static_cast<float>(gx) / static_cast<float>(cfg.grid_cols - 1);
                const int x = static_cast<int>(x0 + fx * static_cast<float>(x1 - x0));

                int best_sad = std::numeric_limits<int>::max();
                int best_dx = 0;
                int best_dy = 0;

                for (int dy = -cfg.search_radius; dy <= cfg.search_radius; ++dy) {
                    for (int dx = -cfg.search_radius; dx <= cfg.search_radius; ++dx) {
                        const int sad = sad_patch(gray_prev.data(), gray_curr.data(),
                                                  cfg.width, cfg.height, x, y,
                                                  dx, dy, cfg.patch_radius);
                        if (sad < best_sad) {
                            best_sad = sad;
                            best_dx = dx;
                            best_dy = dy;
                        }
                    }
                }

                dxs.push_back(static_cast<float>(best_dx));
                dys.push_back(static_cast<float>(best_dy));
            }
        }

        return std::make_pair(median(dxs), median(dys));
    }

    void warp_translate(const uint8_t* input, uint8_t* output, float tx, float ty) const {
        const int w = cfg.width;
        const int h = cfg.height;
        const int ch = cfg.input_channels;

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const float src_x = static_cast<float>(x) - tx;
                const float src_y = static_cast<float>(y) - ty;

                const int x0 = static_cast<int>(std::floor(src_x));
                const int y0 = static_cast<int>(std::floor(src_y));
                const int x1 = x0 + 1;
                const int y1 = y0 + 1;

                const float ax = src_x - static_cast<float>(x0);
                const float ay = src_y - static_cast<float>(y0);

                for (int c = 0; c < ch; ++c) {
                    const float v00 = static_cast<float>(sample_ch(input, w, h, c, ch, x0, y0, cfg.border_mode));
                    const float v01 = static_cast<float>(sample_ch(input, w, h, c, ch, x1, y0, cfg.border_mode));
                    const float v10 = static_cast<float>(sample_ch(input, w, h, c, ch, x0, y1, cfg.border_mode));
                    const float v11 = static_cast<float>(sample_ch(input, w, h, c, ch, x1, y1, cfg.border_mode));

                    const float top = v00 + ax * (v01 - v00);
                    const float bot = v10 + ax * (v11 - v10);
                    const float val = top + ay * (bot - top);

                    output[(y * w + x) * ch + c] = static_cast<uint8_t>(std::max(0.f, std::min(255.f, val)));
                }
            }
        }
    }
};

Stabilizer::Stabilizer(const StabilizerConfig& cfg) : impl_(new Impl(cfg)) {}
Stabilizer::~Stabilizer() = default;

void Stabilizer::reset() {
    impl_->initialized = false;
    impl_->path_x = 0.f;
    impl_->path_y = 0.f;
    impl_->smooth_x = 0.f;
    impl_->smooth_y = 0.f;
}

bool Stabilizer::process(const uint8_t* input, uint8_t* output) {
    if (!input || !output) {
        return false;
    }

    const size_t frame_bytes = static_cast<size_t>(impl_->cfg.width) * static_cast<size_t>(impl_->cfg.height) *
                               static_cast<size_t>(impl_->cfg.input_channels);

    const uint8_t* input_read_ptr = input;
    if (input == output) {
        std::memcpy(impl_->frame_scratch.data(), input, frame_bytes);
        input_read_ptr = impl_->frame_scratch.data();
    }

    impl_->to_gray(input_read_ptr, impl_->gray_curr);

    if (!impl_->initialized) {
        std::memcpy(output, input_read_ptr, frame_bytes);
        impl_->gray_prev.swap(impl_->gray_curr);
        impl_->initialized = true;
        return true;
    }

    const std::pair<float, float> motion = impl_->estimate_motion();
    impl_->path_x += motion.first;
    impl_->path_y += motion.second;

    const float a = std::min(std::max(impl_->cfg.ema_alpha, 0.f), 0.9999f);
    impl_->smooth_x = a * impl_->smooth_x + (1.f - a) * impl_->path_x;
    impl_->smooth_y = a * impl_->smooth_y + (1.f - a) * impl_->path_y;

    const float correction_x = impl_->smooth_x - impl_->path_x;
    const float correction_y = impl_->smooth_y - impl_->path_y;

    impl_->warp_translate(input_read_ptr, output, correction_x, correction_y);
    impl_->gray_prev.swap(impl_->gray_curr);
    return true;
}

} // namespace rtsdk

struct RTSdkStabilizerHandle {
    rtsdk::Stabilizer instance;
    explicit RTSdkStabilizerHandle(const rtsdk::StabilizerConfig& cfg) : instance(cfg) {}
};

extern "C" {

RTSdkStabilizerHandle* rtsdk_create(const struct RTSdkConfig* cfg) {
    if (!cfg) {
        return nullptr;
    }

    rtsdk::StabilizerConfig cpp;
    cpp.width = cfg->width;
    cpp.height = cfg->height;
    cpp.input_channels = cfg->input_channels;
    cpp.search_radius = cfg->search_radius;
    cpp.patch_radius = cfg->patch_radius;
    cpp.grid_cols = cfg->grid_cols;
    cpp.grid_rows = cfg->grid_rows;
    cpp.ema_alpha = cfg->ema_alpha;
    cpp.border_mode = cfg->border_mode;

    if (cpp.width <= 0 || cpp.height <= 0 || (cpp.input_channels != 1 && cpp.input_channels != 3)) {
        return nullptr;
    }

    return new RTSdkStabilizerHandle(cpp);
}

void rtsdk_destroy(RTSdkStabilizerHandle* handle) {
    delete handle;
}

void rtsdk_reset(RTSdkStabilizerHandle* handle) {
    if (handle) {
        handle->instance.reset();
    }
}

int rtsdk_process(RTSdkStabilizerHandle* handle, const uint8_t* input, uint8_t* output) {
    if (!handle) {
        return 0;
    }
    return handle->instance.process(input, output) ? 1 : 0;
}

} // extern "C"
