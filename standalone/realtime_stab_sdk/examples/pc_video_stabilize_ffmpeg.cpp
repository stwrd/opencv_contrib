#include "realtime_stab_sdk.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#define POPEN _popen
#define PCLOSE _pclose
#else
#define POPEN popen
#define PCLOSE pclose
#endif

namespace {

std::string run_command_capture(const std::string& cmd) {
    std::array<char, 256> buf{};
    std::string out;
    FILE* p = POPEN(cmd.c_str(), "r");
    if (!p) return out;
    while (std::fgets(buf.data(), static_cast<int>(buf.size()), p)) {
        out += buf.data();
    }
    PCLOSE(p);
    return out;
}

bool probe_video(const std::string& input, int& w, int& h, double& fps) {
    std::string cmd =
        "ffprobe -v error -select_streams v:0 "
        "-show_entries stream=width,height,avg_frame_rate "
        "-of default=noprint_wrappers=1:nokey=1 \"" + input + "\"";

    const std::string out = run_command_capture(cmd);
    if (out.empty()) return false;

    std::istringstream iss(out);
    std::string line_w, line_h, line_rate;
    if (!std::getline(iss, line_w) || !std::getline(iss, line_h) || !std::getline(iss, line_rate)) {
        return false;
    }

    w = std::atoi(line_w.c_str());
    h = std::atoi(line_h.c_str());

    size_t slash = line_rate.find('/');
    if (slash == std::string::npos) return false;
    const double num = std::atof(line_rate.substr(0, slash).c_str());
    const double den = std::atof(line_rate.substr(slash + 1).c_str());
    fps = (den > 0.0) ? (num / den) : 30.0;

    return w > 0 && h > 0 && fps > 0.0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: pc_video_stabilize_ffmpeg <input_video> <output_video>" << std::endl;
        return 1;
    }

    const std::string input = argv[1];
    const std::string output = argv[2];

    int width = 0;
    int height = 0;
    double fps = 0.0;
    if (!probe_video(input, width, height, fps)) {
        std::cerr << "Failed to probe input video. Please ensure ffprobe is installed." << std::endl;
        return 2;
    }

    rtsdk::StabilizerConfig cfg;
    cfg.width = width;
    cfg.height = height;
    cfg.input_channels = 3;
    cfg.search_radius = 12;
    cfg.patch_radius = 4;
    cfg.grid_cols = 10;
    cfg.grid_rows = 6;
    cfg.ema_alpha = 0.90f;
    cfg.motion_estimator = 1; // KLT optical flow
    cfg.smoothing_mode = 1;   // Gaussian smoothing

    rtsdk::Stabilizer sdk(cfg);

    std::ostringstream dec;
    dec << "ffmpeg -v error -i \"" << input << "\" -f rawvideo -pix_fmt rgb24 -";
    std::ostringstream enc;
    enc << "ffmpeg -y -v error -f rawvideo -pix_fmt rgb24 -s " << width << "x" << height
        << " -r " << fps << " -i - -c:v libx264 -pix_fmt yuv420p \"" << output << "\"";

    FILE* decoder = POPEN(dec.str().c_str(), "r");
    FILE* encoder = POPEN(enc.str().c_str(), "w");
    if (!decoder || !encoder) {
        std::cerr << "Failed to start ffmpeg pipe. Ensure ffmpeg is installed." << std::endl;
        if (decoder) PCLOSE(decoder);
        if (encoder) PCLOSE(encoder);
        return 3;
    }

    const size_t frame_bytes = static_cast<size_t>(width) * static_cast<size_t>(height) * 3u;
    std::vector<uint8_t> in(frame_bytes);
    std::vector<uint8_t> out(frame_bytes);

    while (std::fread(in.data(), 1, frame_bytes, decoder) == frame_bytes) {
        if (!sdk.process(in.data(), out.data())) {
            std::cerr << "Stabilization failed." << std::endl;
            PCLOSE(decoder);
            PCLOSE(encoder);
            return 4;
        }

        if (std::fwrite(out.data(), 1, frame_bytes, encoder) != frame_bytes) {
            std::cerr << "Write to encoder failed." << std::endl;
            PCLOSE(decoder);
            PCLOSE(encoder);
            return 5;
        }
    }

    PCLOSE(decoder);
    PCLOSE(encoder);

    std::cout << "Output written to: " << output << std::endl;
    return 0;
}
