#include "realtime_stab_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
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

inline float sample_gray_bilinear(const uint8_t* gray, int w, int h, float x, float y) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const float ax = x - static_cast<float>(x0);
    const float ay = y - static_cast<float>(y0);

    const auto p = [&](int xx, int yy) -> float {
        const int cx = std::min(std::max(xx, 0), w - 1);
        const int cy = std::min(std::max(yy, 0), h - 1);
        return static_cast<float>(gray[cy * w + cx]);
    };

    const float v00 = p(x0, y0);
    const float v01 = p(x1, y0);
    const float v10 = p(x0, y1);
    const float v11 = p(x1, y1);
    const float top = v00 + ax * (v01 - v00);
    const float bot = v10 + ax * (v11 - v10);
    return top + ay * (bot - top);
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
        cfg.motion_estimator = (cfg.motion_estimator == 0) ? 0 : 1;
        cfg.max_features = std::max(cfg.max_features, 20);
        cfg.klt_win_radius = std::max(cfg.klt_win_radius, 2);
        cfg.klt_max_iters = std::max(cfg.klt_max_iters, 3);
        cfg.klt_epsilon = std::max(cfg.klt_epsilon, 0.0001f);
        cfg.smoothing_mode = (cfg.smoothing_mode == 0) ? 0 : 1;
        cfg.gaussian_radius = std::max(cfg.gaussian_radius, 1);
        if (cfg.gaussian_sigma <= 0.f) {
            cfg.gaussian_sigma = std::max(static_cast<float>(cfg.gaussian_radius) * 0.5f, 1.f);
        }

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
    std::vector<float> path_hist_x;
    std::vector<float> path_hist_y;
    std::vector<float> gauss_weights;

    void init_gaussian_kernel() {
        gauss_weights.resize(static_cast<size_t>(cfg.gaussian_radius + 1), 0.f);
        float sum = 0.f;
        for (int i = 0; i <= cfg.gaussian_radius; ++i) {
            const float x = static_cast<float>(i);
            const float w = std::exp(-(x * x) / (2.f * cfg.gaussian_sigma * cfg.gaussian_sigma));
            gauss_weights[static_cast<size_t>(i)] = w;
            sum += w;
        }
        if (sum > 0.f) {
            for (size_t i = 0; i < gauss_weights.size(); ++i) {
                gauss_weights[i] /= sum;
            }
        }
    }

    std::pair<float, float> smooth_path(float px, float py) {
        path_hist_x.push_back(px);
        path_hist_y.push_back(py);

        if (cfg.smoothing_mode == 0) {
            const float a = std::min(std::max(cfg.ema_alpha, 0.f), 0.9999f);
            smooth_x = a * smooth_x + (1.f - a) * px;
            smooth_y = a * smooth_y + (1.f - a) * py;
            return std::make_pair(smooth_x, smooth_y);
        }

        if (gauss_weights.empty()) {
            init_gaussian_kernel();
        }

        float sx = 0.f;
        float sy = 0.f;
        float sw = 0.f;
        const int n = static_cast<int>(path_hist_x.size()) - 1;
        for (int k = 0; k <= cfg.gaussian_radius; ++k) {
            const int idx = n - k;
            if (idx < 0) break;
            const float w = gauss_weights[static_cast<size_t>(k)];
            sx += path_hist_x[static_cast<size_t>(idx)] * w;
            sy += path_hist_y[static_cast<size_t>(idx)] * w;
            sw += w;
        }
        if (sw > 1e-6f) {
            sx /= sw;
            sy /= sw;
        }
        smooth_x = sx;
        smooth_y = sy;
        return std::make_pair(smooth_x, smooth_y);
    }

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
        if (cfg.motion_estimator == 1) {
            return estimate_motion_klt();
        }
        return estimate_motion_sad();
    }

    std::pair<float, float> estimate_motion_sad() const {
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

    std::pair<float, float> estimate_motion_klt() const {
        struct Pt { float x, y; };
        std::vector<Pt> features;
        features.reserve(static_cast<size_t>(cfg.max_features * 2));

        const int w = cfg.width;
        const int h = cfg.height;
        const int r = cfg.klt_win_radius;
        const int border = std::max(6, r + 2);
        const int step = 4;

        // 1) lightweight Shi-Tomasi feature picking
        std::vector<std::pair<float, Pt>> candidates;
        for (int y = border; y < h - border; y += step) {
            for (int x = border; x < w - border; x += step) {
                float sxx = 0.f, syy = 0.f, sxy = 0.f;
                for (int wy = -1; wy <= 1; ++wy) {
                    for (int wx = -1; wx <= 1; ++wx) {
                        const int xx = x + wx;
                        const int yy = y + wy;
                        const float ix = static_cast<float>(gray_prev[yy * w + (xx + 1)]) -
                                         static_cast<float>(gray_prev[yy * w + (xx - 1)]);
                        const float iy = static_cast<float>(gray_prev[(yy + 1) * w + xx]) -
                                         static_cast<float>(gray_prev[(yy - 1) * w + xx]);
                        sxx += ix * ix;
                        syy += iy * iy;
                        sxy += ix * iy;
                    }
                }
                const float tr = sxx + syy;
                const float det = sxx * syy - sxy * sxy;
                const float disc = std::max(tr * tr - 4.f * det, 0.f);
                const float min_eig = 0.5f * (tr - std::sqrt(disc));
                if (min_eig > 1000.f) {
                    candidates.push_back(std::make_pair(min_eig, Pt{static_cast<float>(x), static_cast<float>(y)}));
                }
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const std::pair<float, Pt>& a, const std::pair<float, Pt>& b) {
                      return a.first > b.first;
                  });

        const float min_dist2 = 9.f * 9.f;
        for (size_t i = 0; i < candidates.size() && static_cast<int>(features.size()) < cfg.max_features; ++i) {
            const Pt p = candidates[i].second;
            bool good = true;
            for (size_t j = 0; j < features.size(); ++j) {
                const float dx = features[j].x - p.x;
                const float dy = features[j].y - p.y;
                if (dx * dx + dy * dy < min_dist2) {
                    good = false;
                    break;
                }
            }
            if (good) features.push_back(p);
        }

        if (features.empty()) return std::make_pair(0.f, 0.f);

        // 2) iterative LK tracking
        std::vector<float> dxs;
        std::vector<float> dys;
        dxs.reserve(features.size());
        dys.reserve(features.size());

        for (size_t i = 0; i < features.size(); ++i) {
            const float x = features[i].x;
            const float y = features[i].y;

            float dx = 0.f, dy = 0.f;
            bool ok = true;

            for (int iter = 0; iter < cfg.klt_max_iters; ++iter) {
                float a00 = 0.f, a01 = 0.f, a11 = 0.f;
                float b0 = 0.f, b1 = 0.f;

                for (int wy = -r; wy <= r; ++wy) {
                    for (int wx = -r; wx <= r; ++wx) {
                        const float px = x + static_cast<float>(wx);
                        const float py = y + static_cast<float>(wy);
                        const float qx = px + dx;
                        const float qy = py + dy;

                        if (qx < 1.f || qx >= static_cast<float>(w - 2) ||
                            qy < 1.f || qy >= static_cast<float>(h - 2) ||
                            px < 1.f || px >= static_cast<float>(w - 2) ||
                            py < 1.f || py >= static_cast<float>(h - 2)) {
                            ok = false;
                            break;
                        }

                        const float i0 = sample_gray_bilinear(gray_prev.data(), w, h, px, py);
                        const float i1 = sample_gray_bilinear(gray_curr.data(), w, h, qx, qy);

                        const float gx = 0.5f * (sample_gray_bilinear(gray_prev.data(), w, h, px + 1.f, py) -
                                                 sample_gray_bilinear(gray_prev.data(), w, h, px - 1.f, py));
                        const float gy = 0.5f * (sample_gray_bilinear(gray_prev.data(), w, h, px, py + 1.f) -
                                                 sample_gray_bilinear(gray_prev.data(), w, h, px, py - 1.f));
                        const float err = i1 - i0;

                        a00 += gx * gx;
                        a01 += gx * gy;
                        a11 += gy * gy;
                        b0 += gx * err;
                        b1 += gy * err;
                    }
                    if (!ok) break;
                }

                if (!ok) break;

                const float det = a00 * a11 - a01 * a01;
                if (det < 1e-4f) {
                    ok = false;
                    break;
                }

                const float inv00 = a11 / det;
                const float inv01 = -a01 / det;
                const float inv11 = a00 / det;
                const float step_x = -(inv00 * b0 + inv01 * b1);
                const float step_y = -(inv01 * b0 + inv11 * b1);
                dx += step_x;
                dy += step_y;

                if (step_x * step_x + step_y * step_y < cfg.klt_epsilon * cfg.klt_epsilon) {
                    break;
                }
            }

            if (ok && std::fabs(dx) < static_cast<float>(cfg.search_radius * 2) &&
                std::fabs(dy) < static_cast<float>(cfg.search_radius * 2)) {
                dxs.push_back(dx);
                dys.push_back(dy);
            }
        }

        if (dxs.empty()) return std::make_pair(0.f, 0.f);
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

Stabilizer::Stabilizer(const StabilizerConfig& cfg) : impl_(new Impl(cfg)) {
    impl_->init_gaussian_kernel();
}
Stabilizer::~Stabilizer() = default;

void Stabilizer::reset() {
    impl_->initialized = false;
    impl_->path_x = 0.f;
    impl_->path_y = 0.f;
    impl_->smooth_x = 0.f;
    impl_->smooth_y = 0.f;
    impl_->path_hist_x.clear();
    impl_->path_hist_y.clear();
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

    const std::pair<float, float> smooth = impl_->smooth_path(impl_->path_x, impl_->path_y);
    const float correction_x = smooth.first - impl_->path_x;
    const float correction_y = smooth.second - impl_->path_y;

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
    cpp.motion_estimator = cfg->motion_estimator;
    cpp.max_features = cfg->max_features;
    cpp.klt_win_radius = cfg->klt_win_radius;
    cpp.klt_max_iters = cfg->klt_max_iters;
    cpp.klt_epsilon = cfg->klt_epsilon;
    cpp.smoothing_mode = cfg->smoothing_mode;
    cpp.gaussian_radius = cfg->gaussian_radius;
    cpp.gaussian_sigma = cfg->gaussian_sigma;

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
