#include "realtime_stab_sdk.hpp"

#include <fstream>
#include <iostream>
#include <vector>

// Demo for raw frame stream (e.g. Y8 or RGB24 dump).
int main() {
    rtsdk::StabilizerConfig cfg;
    cfg.width = 640;
    cfg.height = 360;
    cfg.input_channels = 1;
    cfg.motion_estimator = 1; // KLT optical flow
    cfg.smoothing_mode = 1;   // Gaussian smoothing

    rtsdk::Stabilizer sdk(cfg);

    const size_t bytes = static_cast<size_t>(cfg.width) * static_cast<size_t>(cfg.height) *
                         static_cast<size_t>(cfg.input_channels);
    std::vector<uint8_t> input(bytes);
    std::vector<uint8_t> output(bytes);

    std::ifstream fin("input.y8", std::ios::binary);
    std::ofstream fout("output.y8", std::ios::binary);
    if (!fin || !fout) {
        std::cerr << "Please place input.y8 beside executable" << std::endl;
        return 1;
    }

    while (fin.read(reinterpret_cast<char*>(input.data()), static_cast<std::streamsize>(bytes))) {
        if (!sdk.process(input.data(), output.data())) {
            std::cerr << "process() failed" << std::endl;
            return 2;
        }
        fout.write(reinterpret_cast<const char*>(output.data()), static_cast<std::streamsize>(bytes));
    }

    std::cout << "done" << std::endl;
    return 0;
}
