#include "realtime_stab_sdk.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace rtsdk {
namespace {

struct Motion2D {
    // [a00 a01 tx; a10 a11 ty]
    float a00, a01, tx;
    float a10, a11, ty;
};

inline Motion2D identity_motion() {
    Motion2D m;
    m.a00 = 1.f; m.a01 = 0.f; m.tx = 0.f;
    m.a10 = 0.f; m.a11 = 1.f; m.ty = 0.f;
    return m;
}

inline Motion2D compose_motion(const Motion2D& A, const Motion2D& B) {
    // A * B
    Motion2D o;
    o.a00 = A.a00 * B.a00 + A.a01 * B.a10;
    o.a01 = A.a00 * B.a01 + A.a01 * B.a11;
    o.tx  = A.a00 * B.tx  + A.a01 * B.ty + A.tx;
    o.a10 = A.a10 * B.a00 + A.a11 * B.a10;
    o.a11 = A.a10 * B.a01 + A.a11 * B.a11;
    o.ty  = A.a10 * B.tx  + A.a11 * B.ty + A.ty;
    return o;
}

inline bool invert_motion(const Motion2D& M, Motion2D& inv) {
    const float det = M.a00 * M.a11 - M.a01 * M.a10;
    if (std::fabs(det) < 1e-7f) return false;
    const float id = 1.f / det;

    inv.a00 =  M.a11 * id;
    inv.a01 = -M.a01 * id;
    inv.a10 = -M.a10 * id;
    inv.a11 =  M.a00 * id;
    inv.tx = -(inv.a00 * M.tx + inv.a01 * M.ty);
    inv.ty = -(inv.a10 * M.tx + inv.a11 * M.ty);
    return true;
}

inline int sample_ch(const uint8_t* frame, int w, int h, int c, int channels,
                     int x, int y, int border_mode) {
    if (x >= 0 && x < w && y >= 0 && y < h) {
        return frame[(y * w + x) * channels + c];
    }
    if (border_mode == 0) return 0;
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

inline bool solve_linear_6x6(float A[6][6], float b[6], float x[6]) {
    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        float maxv = std::fabs(A[col][col]);
        for (int r = col + 1; r < 6; ++r) {
            const float v = std::fabs(A[r][col]);
            if (v > maxv) {
                maxv = v;
                pivot = r;
            }
        }
        if (maxv < 1e-7f) return false;
        if (pivot != col) {
            for (int c = col; c < 6; ++c) std::swap(A[col][c], A[pivot][c]);
            std::swap(b[col], b[pivot]);
        }

        const float inv = 1.f / A[col][col];
        for (int c = col; c < 6; ++c) A[col][c] *= inv;
        b[col] *= inv;

        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            const float f = A[r][col];
            for (int c = col; c < 6; ++c) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int i = 0; i < 6; ++i) x[i] = b[i];
    return true;
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
        cfg.motion_model = (cfg.motion_model == 0) ? 0 : 1;
        cfg.trim_ratio = std::min(std::max(cfg.trim_ratio, 0.f), 0.45f);
        cfg.latency_radius = std::max(cfg.latency_radius, 0);
        cfg.gaussian_radius = std::max(cfg.gaussian_radius, 1);
        cfg.search_radius = std::max(cfg.search_radius, 1);
        cfg.grid_cols = std::max(cfg.grid_cols, 1);
        cfg.grid_rows = std::max(cfg.grid_rows, 1);
        cfg.ema_alpha = std::min(std::max(cfg.ema_alpha, 0.f), 0.9999f);
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

    std::vector<Motion2D> cumulative_hist;
    std::vector<std::vector<uint8_t> > frame_hist;
    int base_frame_idx = 0;
    int latest_frame_idx = -1;
    int last_emitted_idx = -1;
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
            for (size_t i = 0; i < gauss_weights.size(); ++i) gauss_weights[i] /= sum;
        }
    }

    float get_param(const Motion2D& m, int idx) const {
        switch (idx) {
            case 0: return m.a00;
            case 1: return m.a01;
            case 2: return m.tx;
            case 3: return m.a10;
            case 4: return m.a11;
            default: return m.ty;
        }
    }

    void set_param(Motion2D& m, int idx, float v) const {
        switch (idx) {
            case 0: m.a00 = v; break;
            case 1: m.a01 = v; break;
            case 2: m.tx = v; break;
            case 3: m.a10 = v; break;
            case 4: m.a11 = v; break;
            default: m.ty = v; break;
        }
    }

    Motion2D smooth_cumulative_motion_at(int target_offset) const {
        Motion2D out = identity_motion();
        if (target_offset < 0 || target_offset >= static_cast<int>(cumulative_hist.size())) {
            return out;
        }

        if (cfg.smoothing_mode == 0) {
            const float a = cfg.ema_alpha;
            Motion2D sm = identity_motion();
            for (int i = 0; i <= target_offset; ++i) {
                sm.a00 = a * sm.a00 + (1.f - a) * cumulative_hist[static_cast<size_t>(i)].a00;
                sm.a01 = a * sm.a01 + (1.f - a) * cumulative_hist[static_cast<size_t>(i)].a01;
                sm.tx  = a * sm.tx  + (1.f - a) * cumulative_hist[static_cast<size_t>(i)].tx;
                sm.a10 = a * sm.a10 + (1.f - a) * cumulative_hist[static_cast<size_t>(i)].a10;
                sm.a11 = a * sm.a11 + (1.f - a) * cumulative_hist[static_cast<size_t>(i)].a11;
                sm.ty  = a * sm.ty  + (1.f - a) * cumulative_hist[static_cast<size_t>(i)].ty;
            }
            return sm;
        }

        if (gauss_weights.empty()) {
            const_cast<Impl*>(this)->init_gaussian_kernel();
        }

        const int radius = cfg.gaussian_radius;
        const bool use_future = cfg.latency_radius > 0;
        for (int p = 0; p < 6; ++p) {
            float s = 0.f;
            float sw = 0.f;
            if (use_future) {
                for (int d = -radius; d <= radius; ++d) {
                    const int idx = target_offset + d;
                    if (idx < 0 || idx >= static_cast<int>(cumulative_hist.size())) continue;
                    const float w = gauss_weights[static_cast<size_t>(std::abs(d))];
                    s += get_param(cumulative_hist[static_cast<size_t>(idx)], p) * w;
                    sw += w;
                }
            } else {
                for (int d = 0; d <= radius; ++d) {
                    const int idx = target_offset - d;
                    if (idx < 0) break;
                    const float w = gauss_weights[static_cast<size_t>(d)];
                    s += get_param(cumulative_hist[static_cast<size_t>(idx)], p) * w;
                    sw += w;
                }
            }
            if (sw > 1e-6f) s /= sw;
            set_param(out, p, s);
        }
        return out;
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

    Motion2D estimate_motion() const {
        if (cfg.motion_estimator == 1) return estimate_motion_klt();
        return estimate_motion_sad();
    }

    Motion2D estimate_motion_sad() const {
        std::vector<float> dxs;
        std::vector<float> dys;
        dxs.reserve(static_cast<size_t>(cfg.grid_cols * cfg.grid_rows));
        dys.reserve(static_cast<size_t>(cfg.grid_cols * cfg.grid_rows));

        const int margin = std::max(cfg.patch_radius + cfg.search_radius + 1, 8);
        const int x0 = margin, y0 = margin;
        const int x1 = cfg.width - margin - 1;
        const int y1 = cfg.height - margin - 1;
        if (x1 <= x0 || y1 <= y0) return identity_motion();

        for (int gy = 0; gy < cfg.grid_rows; ++gy) {
            const float fy = (cfg.grid_rows == 1) ? 0.f : static_cast<float>(gy) / static_cast<float>(cfg.grid_rows - 1);
            const int y = static_cast<int>(y0 + fy * static_cast<float>(y1 - y0));
            for (int gx = 0; gx < cfg.grid_cols; ++gx) {
                const float fx = (cfg.grid_cols == 1) ? 0.f : static_cast<float>(gx) / static_cast<float>(cfg.grid_cols - 1);
                const int x = static_cast<int>(x0 + fx * static_cast<float>(x1 - x0));

                int best_sad = std::numeric_limits<int>::max();
                int best_dx = 0, best_dy = 0;
                for (int dy = -cfg.search_radius; dy <= cfg.search_radius; ++dy) {
                    for (int dx = -cfg.search_radius; dx <= cfg.search_radius; ++dx) {
                        const int sad = sad_patch(gray_prev.data(), gray_curr.data(), cfg.width, cfg.height,
                                                  x, y, dx, dy, cfg.patch_radius);
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

        Motion2D m = identity_motion();
        m.tx = median(dxs);
        m.ty = median(dys);
        return m;
    }

    bool estimate_affine_from_matches(const std::vector<float>& xs,
                                      const std::vector<float>& ys,
                                      const std::vector<float>& us,
                                      const std::vector<float>& vs,
                                      Motion2D& out) const {
        if (xs.size() < 6U) return false;

        float ATA[6][6] = {{0}};
        float ATb[6] = {0};

        for (size_t i = 0; i < xs.size(); ++i) {
            const float x = xs[i], y = ys[i];
            const float u = us[i], v = vs[i];

            const float r1[6] = {x, y, 1.f, 0.f, 0.f, 0.f};
            const float r2[6] = {0.f, 0.f, 0.f, x, y, 1.f};

            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) {
                    ATA[r][c] += r1[r] * r1[c] + r2[r] * r2[c];
                }
                ATb[r] += r1[r] * u + r2[r] * v;
            }
        }

        float sol[6] = {0};
        if (!solve_linear_6x6(ATA, ATb, sol)) return false;

        out.a00 = sol[0]; out.a01 = sol[1]; out.tx = sol[2];
        out.a10 = sol[3]; out.a11 = sol[4]; out.ty = sol[5];

        const float det = out.a00 * out.a11 - out.a01 * out.a10;
        if (std::fabs(det) < 1e-5f) return false;
        return true;
    }

    Motion2D estimate_motion_klt() const {
        struct Pt { float x, y; };
        std::vector<Pt> features;
        features.reserve(static_cast<size_t>(cfg.max_features * 2));

        const int w = cfg.width;
        const int h = cfg.height;
        const int r = cfg.klt_win_radius;
        const int border = std::max(6, r + 2);

        std::vector<std::pair<float, Pt>> candidates;
        for (int y = border; y < h - border; y += 4) {
            for (int x = border; x < w - border; x += 4) {
                float sxx = 0.f, syy = 0.f, sxy = 0.f;
                for (int wy = -1; wy <= 1; ++wy) {
                    for (int wx = -1; wx <= 1; ++wx) {
                        const int xx = x + wx;
                        const int yy = y + wy;
                        const float ix = static_cast<float>(gray_prev[yy * w + (xx + 1)]) -
                                         static_cast<float>(gray_prev[yy * w + (xx - 1)]);
                        const float iy = static_cast<float>(gray_prev[(yy + 1) * w + xx]) -
                                         static_cast<float>(gray_prev[(yy - 1) * w + xx]);
                        sxx += ix * ix; syy += iy * iy; sxy += ix * iy;
                    }
                }
                const float tr = sxx + syy;
                const float det = sxx * syy - sxy * sxy;
                const float disc = std::max(tr * tr - 4.f * det, 0.f);
                const float min_eig = 0.5f * (tr - std::sqrt(disc));
                if (min_eig > 1000.f) {
                    Pt p = {static_cast<float>(x), static_cast<float>(y)};
                    candidates.push_back(std::make_pair(min_eig, p));
                }
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const std::pair<float, Pt>& a, const std::pair<float, Pt>& b){ return a.first > b.first; });

        const float min_dist2 = 81.f;
        for (size_t i = 0; i < candidates.size() && static_cast<int>(features.size()) < cfg.max_features; ++i) {
            Pt p = candidates[i].second;
            bool good = true;
            for (size_t j = 0; j < features.size(); ++j) {
                const float dx = features[j].x - p.x;
                const float dy = features[j].y - p.y;
                if (dx * dx + dy * dy < min_dist2) { good = false; break; }
            }
            if (good) features.push_back(p);
        }

        if (features.empty()) return identity_motion();

        std::vector<float> dxs, dys;
        std::vector<float> xs, ys, us, vs;
        dxs.reserve(features.size()); dys.reserve(features.size());
        xs.reserve(features.size()); ys.reserve(features.size()); us.reserve(features.size()); vs.reserve(features.size());

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
                if (det < 1e-4f) { ok = false; break; }

                const float inv00 = a11 / det;
                const float inv01 = -a01 / det;
                const float inv11 = a00 / det;
                const float step_x = -(inv00 * b0 + inv01 * b1);
                const float step_y = -(inv01 * b0 + inv11 * b1);
                dx += step_x;
                dy += step_y;

                if (step_x * step_x + step_y * step_y < cfg.klt_epsilon * cfg.klt_epsilon) break;
            }

            if (ok && std::fabs(dx) < static_cast<float>(cfg.search_radius * 2) &&
                std::fabs(dy) < static_cast<float>(cfg.search_radius * 2)) {
                dxs.push_back(dx);
                dys.push_back(dy);
                xs.push_back(x);
                ys.push_back(y);
                us.push_back(x + dx);
                vs.push_back(y + dy);
            }
        }

        if (dxs.empty()) return identity_motion();

        if (cfg.motion_model == 1) {
            Motion2D A;
            if (estimate_affine_from_matches(xs, ys, us, vs, A)) {
                return A;
            }
        }

        Motion2D t = identity_motion();
        t.tx = median(dxs);
        t.ty = median(dys);
        return t;
    }

    void warp_affine(const uint8_t* input, uint8_t* output, const Motion2D& M) const {
        const int w = cfg.width;
        const int h = cfg.height;
        const int ch = cfg.input_channels;
        const float trim = cfg.trim_ratio;
        const float scale = 1.f - 2.f * trim;
        const float off_x = trim * static_cast<float>(w);
        const float off_y = trim * static_cast<float>(h);

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                float src_x = M.a00 * static_cast<float>(x) + M.a01 * static_cast<float>(y) + M.tx;
                float src_y = M.a10 * static_cast<float>(x) + M.a11 * static_cast<float>(y) + M.ty;

                if (trim > 0.f) {
                    src_x = off_x + scale * src_x;
                    src_y = off_y + scale * src_y;
                }

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

    bool emit_frame(int target_idx, uint8_t* output) {
        const int target_off = target_idx - base_frame_idx;
        if (target_off < 0 || target_off >= static_cast<int>(frame_hist.size())) {
            return false;
        }

        const Motion2D& target_cum = cumulative_hist[static_cast<size_t>(target_off)];
        Motion2D inv_target;
        if (!invert_motion(target_cum, inv_target)) {
            inv_target = identity_motion();
        }
        const Motion2D smooth_cum = smooth_cumulative_motion_at(target_off);
        const Motion2D correction = compose_motion(smooth_cum, inv_target);
        warp_affine(frame_hist[static_cast<size_t>(target_off)].data(), output, correction);
        last_emitted_idx = target_idx;
        return true;
    }

    void prune_history_after_emit() {
        const int latency = cfg.latency_radius;
        const int keep_from_idx = (latency > 0) ? (last_emitted_idx - latency) : last_emitted_idx;
        if (keep_from_idx > base_frame_idx) {
            const int drop = keep_from_idx - base_frame_idx;
            frame_hist.erase(frame_hist.begin(), frame_hist.begin() + drop);
            cumulative_hist.erase(cumulative_hist.begin(), cumulative_hist.begin() + drop);
            base_frame_idx = keep_from_idx;
        }
    }
};

Stabilizer::Stabilizer(const StabilizerConfig& cfg) : impl_(new Impl(cfg)) {
    impl_->init_gaussian_kernel();
}
Stabilizer::~Stabilizer() = default;

void Stabilizer::reset() {
    impl_->initialized = false;
    impl_->cumulative_hist.clear();
    impl_->frame_hist.clear();
    impl_->base_frame_idx = 0;
    impl_->latest_frame_idx = -1;
    impl_->last_emitted_idx = -1;
}

bool Stabilizer::process(const uint8_t* input, uint8_t* output) {
    if (!input || !output) return false;

    const size_t frame_bytes = static_cast<size_t>(impl_->cfg.width) * static_cast<size_t>(impl_->cfg.height) *
                               static_cast<size_t>(impl_->cfg.input_channels);

    const uint8_t* input_read_ptr = input;
    if (input == output) {
        std::memcpy(impl_->frame_scratch.data(), input, frame_bytes);
        input_read_ptr = impl_->frame_scratch.data();
    }

    impl_->to_gray(input_read_ptr, impl_->gray_curr);

    std::vector<uint8_t> cur_frame(frame_bytes);
    std::memcpy(cur_frame.data(), input_read_ptr, frame_bytes);

    if (!impl_->initialized) {
        impl_->initialized = true;
        impl_->latest_frame_idx = 0;
        impl_->base_frame_idx = 0;
        impl_->frame_hist.push_back(cur_frame);
        impl_->cumulative_hist.push_back(identity_motion());
        std::memcpy(output, input_read_ptr, frame_bytes);
        impl_->last_emitted_idx = 0;
        impl_->gray_prev.swap(impl_->gray_curr);
        return true;
    }

    const Motion2D motion = impl_->estimate_motion();
    const Motion2D prev_cum = impl_->cumulative_hist.back();
    const Motion2D cur_cum = compose_motion(motion, prev_cum);

    impl_->latest_frame_idx++;
    impl_->frame_hist.push_back(cur_frame);
    impl_->cumulative_hist.push_back(cur_cum);

    const int latency = impl_->cfg.latency_radius;
    if (latency > 0 && impl_->latest_frame_idx < 2 * latency) {
        std::memcpy(output, input_read_ptr, frame_bytes);
        impl_->gray_prev.swap(impl_->gray_curr);
        return true;
    }

    const int target_idx = (latency > 0) ? (impl_->latest_frame_idx - latency) : impl_->latest_frame_idx;
    if (!impl_->emit_frame(target_idx, output)) {
        std::memcpy(output, input_read_ptr, frame_bytes);
        impl_->gray_prev.swap(impl_->gray_curr);
        return true;
    }

    impl_->prune_history_after_emit();

    impl_->gray_prev.swap(impl_->gray_curr);
    return true;
}

bool Stabilizer::flush(uint8_t* output) {
    if (!output || !impl_->initialized) return false;
    if (impl_->cfg.latency_radius <= 0) return false;

    const int next_idx = impl_->last_emitted_idx + 1;
    if (next_idx > impl_->latest_frame_idx) return false;
    if (!impl_->emit_frame(next_idx, output)) return false;
    impl_->prune_history_after_emit();
    return true;
}

} // namespace rtsdk

struct RTSdkStabilizerHandle {
    rtsdk::Stabilizer instance;
    explicit RTSdkStabilizerHandle(const rtsdk::StabilizerConfig& cfg) : instance(cfg) {}
};

extern "C" {

RTSdkStabilizerHandle* rtsdk_create(const struct RTSdkConfig* cfg) {
    if (!cfg) return nullptr;

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
    cpp.motion_model = cfg->motion_model;
    cpp.trim_ratio = cfg->trim_ratio;
    cpp.latency_radius = cfg->latency_radius;

    if (cpp.width <= 0 || cpp.height <= 0 || (cpp.input_channels != 1 && cpp.input_channels != 3)) {
        return nullptr;
    }

    return new RTSdkStabilizerHandle(cpp);
}

void rtsdk_destroy(RTSdkStabilizerHandle* handle) { delete handle; }

void rtsdk_reset(RTSdkStabilizerHandle* handle) {
    if (handle) handle->instance.reset();
}

int rtsdk_process(RTSdkStabilizerHandle* handle, const uint8_t* input, uint8_t* output) {
    if (!handle) return 0;
    return handle->instance.process(input, output) ? 1 : 0;
}

int rtsdk_flush(RTSdkStabilizerHandle* handle, uint8_t* output) {
    if (!handle) return 0;
    return handle->instance.flush(output) ? 1 : 0;
}

} // extern "C"
