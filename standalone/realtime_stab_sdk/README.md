# Real-time Stabilization SDK (Standalone, No OpenCV Dependency)

这个 SDK 是一个独立 C++ 实时防抖核心，不依赖 OpenCV。你可以直接编译成 `.a/.so`（Android）或 `.a/.framework`（iOS）后接入 App。

## 特性

- 纯 C++14 实现，不依赖 OpenCV。
- 同一套核心算法可在 Android / iOS 共用。
- 提供 C++ API + C API（适合 JNI / ObjC / Swift 桥接）。
- 轻量、实时、流式：每帧调用一次 `process()`。

## 算法说明（OnePass 思路的独立化简版本）

1. 灰度化输入帧（可直接传灰度）。
2. 使用网格点 + 局部块匹配（SAD）估计相邻帧全局平移。
3. 对累计轨迹做 EMA 平滑，得到稳定轨迹。
4. 用平滑补偿量做亚像素平移重采样输出。

> 该版本侧重“低依赖 + 高可移植 + 易接入”。

## 编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

默认会编译两个示例：
- `rtsdk_example`：原始帧流示例
- `rtsdk_pc_video_test`：PC 端视频输入/输出示例（依赖系统安装 `ffmpeg/ffprobe` 可执行文件）

## 集成建议（移动端）

- 每路视频流维护一个 `Stabilizer` 实例。
- 分辨率变化、相机重启时调用 `reset()`。
- 推荐首参数：
  - `search_radius=12`
  - `patch_radius=4`
  - `grid_cols=10`, `grid_rows=6`
  - `ema_alpha=0.90`

## 高性能编译建议

- Android NDK / Clang：`-O3 -ffast-math -fvisibility=hidden`
- iOS / clang：`-O3 -ffast-math`
- 生产环境建议 `arm64` 优先。

## API 快速示例

```cpp
rtsdk::StabilizerConfig cfg;
cfg.width = 1280;
cfg.height = 720;
cfg.input_channels = 3;

rtsdk::Stabilizer sdk(cfg);
sdk.process(input_ptr, output_ptr);
```

## PC 端测试示例（输入视频 -> 输出稳定视频）

构建后执行：

```bash
./build/rtsdk_pc_video_test input.mp4 stabilized.mp4
```

该示例会：
1. 用 `ffprobe` 自动获取输入视频宽高和帧率；
2. 用 `ffmpeg` 管道把输入解码为 `rgb24` 原始帧；
3. 调用 SDK 逐帧稳定；
4. 通过 `ffmpeg` 编码输出稳定视频（H.264 / yuv420p）。
