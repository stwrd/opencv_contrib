#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace rtsdk {

struct StabilizerConfig {
    int width = 1280;
    int height = 720;
    int input_channels = 3;       // 1 = Gray, 3 = RGB/BGR
    int search_radius = 12;       // pixel range for block matching
    int patch_radius = 4;         // patch half size for SAD
    int grid_cols = 10;           // tracking grid columns
    int grid_rows = 6;            // tracking grid rows
    float ema_alpha = 0.90f;      // smooth camera path: larger -> smoother
    int border_mode = 1;          // 0=zero, 1=replicate
    int motion_estimator = 1;     // 0=SAD block-matching, 1=KLT optical-flow (recommended)
    int max_features = 200;       // KLT: max tracked features
    int klt_win_radius = 4;       // KLT: patch radius
    int klt_max_iters = 10;       // KLT: iterations per feature
    float klt_epsilon = 0.01f;    // KLT: convergence threshold
    int smoothing_mode = 1;       // 0=EMA, 1=Gaussian (closer to OnePass)
    int gaussian_radius = 15;     // Gaussian smoothing radius
    float gaussian_sigma = -1.f;  // Gaussian sigma (<0 => auto)
    int motion_model = 1;         // 0=Translation, 1=Affine
    float trim_ratio = 0.04f;     // post-crop ratio [0, 0.45] to suppress border artifacts
    int latency_radius = 0;       // 0=low-latency, >0 enables delayed output for stronger stability
};

class Stabilizer {
public:
    explicit Stabilizer(const StabilizerConfig& cfg);
    ~Stabilizer();

    Stabilizer(const Stabilizer&) = delete;
    Stabilizer& operator=(const Stabilizer&) = delete;

    void reset();

    // Input/output buffers are tightly packed HWC (uint8)
    // output can alias input for in-place processing.
    bool process(const uint8_t* input, uint8_t* output);
    // Flush delayed frames when latency_radius > 0. Returns false when queue is empty.
    bool flush(uint8_t* output);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rtsdk

extern "C" {

typedef struct RTSdkStabilizerHandle RTSdkStabilizerHandle;

struct RTSdkConfig {
    int width;
    int height;
    int input_channels;
    int search_radius;
    int patch_radius;
    int grid_cols;
    int grid_rows;
    float ema_alpha;
    int border_mode;
    int motion_estimator;
    int max_features;
    int klt_win_radius;
    int klt_max_iters;
    float klt_epsilon;
    int smoothing_mode;
    int gaussian_radius;
    float gaussian_sigma;
    int motion_model;
    float trim_ratio;
    int latency_radius;
};

RTSdkStabilizerHandle* rtsdk_create(const struct RTSdkConfig* cfg);
void rtsdk_destroy(RTSdkStabilizerHandle* handle);
void rtsdk_reset(RTSdkStabilizerHandle* handle);
int rtsdk_process(RTSdkStabilizerHandle* handle, const uint8_t* input, uint8_t* output);
int rtsdk_flush(RTSdkStabilizerHandle* handle, uint8_t* output);

}
