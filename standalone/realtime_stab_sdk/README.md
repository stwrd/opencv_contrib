# Real-time Stabilization SDK (Standalone, No OpenCV Dependency)

这个 SDK 是一个独立 C++ 实时防抖核心，不依赖 OpenCV。你可以直接编译成 `.a/.so`（Android）或 `.a/.framework`（iOS）后接入 App。

## 特性

- 纯 C++14 实现，不依赖 OpenCV。
- 同一套核心算法可在 Android / iOS 共用。
- 提供 C++ API + C API（适合 JNI / ObjC / Swift 桥接）。
- 轻量、实时、流式：每帧调用一次 `process()`。

## 算法说明（OnePass 思路的独立化简版本）

1. 灰度化输入帧（可直接传灰度）。
2. 使用 KLT/SAD 估计相邻帧运动（支持平移 / 仿射模型，含残差中值外点剔除）。
3. 对累计轨迹的 `[x, y, angle]` 做平滑（支持 EMA / Gaussian，默认 Gaussian 更接近 OnePass），并抑制微小 scale 噪声。
4. 用平滑补偿量做亚像素仿射重采样输出，并支持 `trim_ratio` 中心裁切抑制黑边。

> 该版本侧重“低依赖 + 高可移植 + 易接入”。
>
> 当前默认使用 **KLT 光流**（`motion_estimator=1`），SAD 作为备选（`motion_estimator=0`）。
>
> 轨迹平滑默认使用 **Gaussian**（`smoothing_mode=1`），EMA 可选（`smoothing_mode=0`）。
>
> 运动模型默认使用 **Affine**（`motion_model=1`），可切回平移模型（`motion_model=0`）。
>
> 默认 `latency_radius=15`（更稳）；实时预览可改为 `0` 降低时延。
>
> 若使用延迟模式，输入结束后请调用 `flush()` / `rtsdk_flush()` 取出尾部缓存帧。
>
> 若你希望导出完自动清理状态，可使用 `flushAndReset()` / `rtsdk_flush_and_reset()`。

## 编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

默认会编译两个示例：
- `rtsdk_example`：原始帧流示例
- `rtsdk_pc_video_test`：PC 端视频输入/输出示例（依赖系统安装 `ffmpeg/ffprobe` 可执行文件）
- `rtsdk_stream_playback_sim`：模拟“持续播放”队列（约 0.5s 延迟）并流式写出视频

如果你只想产出“可集成库”（不编译示例），加下面两个选项：

```bash
-DRTSDK_BUILD_EXAMPLE=OFF -DRTSDK_BUILD_PC_VIDEO_TEST=OFF
```

另外，`realtime_stab_sdk` 支持 `BUILD_SHARED_LIBS`：
- 默认 `OFF`：生成静态库（`.a` / `.lib`）
- 设置 `-DBUILD_SHARED_LIBS=ON`：生成动态库（`.so` / `.dylib` / `.dll`）

## Android 库构建（可直接集成）

下面示例会产出可供 Android App 集成的 `arm64-v8a` 库：

```bash
cmake -S . -B build-android-arm64 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=21 \
  -DBUILD_SHARED_LIBS=ON \
  -DRTSDK_BUILD_EXAMPLE=OFF \
  -DRTSDK_BUILD_PC_VIDEO_TEST=OFF \
  -DCMAKE_INSTALL_PREFIX=$PWD/install-android-arm64

cmake --build build-android-arm64 -j
cmake --install build-android-arm64
```

构建产物通常在：
- 库：`install-android-arm64/lib/`
- 头文件：`install-android-arm64/include/realtime_stab_sdk.hpp`

如果要支持多架构（如 `armeabi-v7a` / `x86_64`），分别改 `ANDROID_ABI` 各编一份。

## iOS 库构建（可直接集成）

> 需要在 macOS + Xcode 环境执行。

### 1) 真机（arm64）

```bash
cmake -S . -B build-ios-device \
  -G Xcode \
  -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DRTSDK_BUILD_EXAMPLE=OFF \
  -DRTSDK_BUILD_PC_VIDEO_TEST=OFF \
  -DCMAKE_INSTALL_PREFIX=$PWD/install-ios-device

cmake --build build-ios-device --config Release
cmake --install build-ios-device --config Release
```

### 2) 模拟器（arm64 / x86_64，按需）

```bash
cmake -S . -B build-ios-sim \
  -G Xcode \
  -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_SYSROOT=iphonesimulator \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DRTSDK_BUILD_EXAMPLE=OFF \
  -DRTSDK_BUILD_PC_VIDEO_TEST=OFF \
  -DCMAKE_INSTALL_PREFIX=$PWD/install-ios-sim

cmake --build build-ios-sim --config Release
cmake --install build-ios-sim --config Release
```

你可以把 device + simulator 的静态库再合成 `xcframework` 后集成到 iOS 工程。

## 集成建议（移动端）

- 每路视频流维护一个 `Stabilizer` 实例。
- 分辨率变化、相机重启时调用 `reset()`。
- 推荐首参数：
  - `search_radius=12`
  - `patch_radius=4`
  - `grid_cols=10`, `grid_rows=6`
  - `ema_alpha=0.90`
  - `motion_estimator=1`（KLT）
  - `max_features=200`, `klt_win_radius=4`, `klt_max_iters=10`
  - `smoothing_mode=1`（Gaussian）
  - `gaussian_radius=15`, `gaussian_sigma=-1(自动)`
  - `motion_model=1`（Affine）
  - `trim_ratio=0.03~0.06`（根据画面边缘黑边情况调节）
  - `latency_radius=15`（默认更稳）/ `0`（实时预览低时延）

离线导出流程建议：
1. 按帧调用 `process()`;
2. 输入结束后循环调用 `flush()` 直到返回 `false`。

或使用便捷模式：循环调用 `flushAndReset()`，最后一次会自动 `reset()`。

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

## PC 端“持续播放”仿真示例（输入视频 -> 延迟队列 -> 输出视频）

构建后执行：

```bash
./build/rtsdk_stream_playback_sim input.mp4 stream_sim_out.mp4
```

该示例会：
1. 自动读取输入 fps；
2. 设置 `latency_radius ≈ 0.5 * fps`；
3. 按帧调用 `process()`，放入播放队列；
4. 队列达到阈值后持续写出（模拟稳定播放）；
5. 结束时 `flushAndReset()` 输出剩余延迟帧。
