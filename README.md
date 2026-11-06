# RK3588 双海康工业相机视频拼接

两台固定安装、有重叠视野的海康机器人面阵相机分别采集左右画面。程序配对双路帧，用单应矩阵把右图映射到左图坐标系，对重叠区域做加权融合，输出一条视频流。适合从固定平面观察产线、传送带或工装。场景深度差越大，越容易在接缝看到视差。

`PairStream` 和 `StitchStream` 使用 [lystream 的 `Stream` / `StreamControl` 接口](https://github.com/PT233/lystream/tree/7f12d207a7514863ac6d225c504a83409d17882c)，实现保存在 `src/lystream_port.cpp`，可在无 CUDA 的机器上构建。每路采集独占一个线程，队列各最多保留四帧。默认按主机收帧时间配对，超过 15 ms 时丢弃较旧帧。配对、超差丢帧和队列溢出计数打印到标准错误。

## 模拟双摄

[查看模拟拼接视频](demo/stitched.mp4)。成片为 1152 × 352、30 fps、4 秒，包含一条 H.264 视频轨。

需要 CMake 3.18+、Git 和 C++17 编译器。构建会拉取固定版本的 lystream 接口，也可用 `-DLYSTREAM_SOURCE_DIR=/path/to/lystream` 指定已有检出。Windows 若使用的 Ninja 不支持中文路径，可把源码和构建目录放在纯 ASCII 路径下。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/dual-hik-stitch --synthetic --calibration config/synthetic.calib \
  --frames 120 --output stitched.bgr
ffmpeg -f rawvideo -pixel_format bgr24 -video_size 1152x352 -framerate 30 \
  -i stitched.bgr -an -c:v libx264 -pix_fmt yuv420p stitched.mp4
```

Windows PowerShell 可分别运行以上命令，单行填写参数，并将可执行文件路径改为 `./build/dual-hik-stitch.exe`，多配置生成器通常放在 `./build/Release/`。模拟相机拍到的是同一移动网格的两个区域，右相机额外有 8 像素垂直偏移。拼接后网格应跨过接缝，红色目标经过接缝时应保持连续。

## RK3588 实机接入

安装海康机器人的 Linux aarch64 MVS SDK。用 MVS 客户端确认两台相机能分别取图，设置一致的帧率、曝光和适合的像素格式。GigE 相机经交换机接入时，交换机到 RK3588 的上联需要承载两台相机的原始流量。可先降低分辨率和帧率，验证连接与拼接后再提升。下载入口见[海康机器人机器视觉下载中心](https://www.hikrobotics.com/en/machinevision/service/download/)。

```sh
cmake -S . -B build-mvs -DCMAKE_BUILD_TYPE=Release -DENABLE_MVS=ON -DMVS_ROOT=/opt/MVS
cmake --build build-mvs
./build-mvs/dual-hik-stitch --list
./build-mvs/dual-hik-stitch --left LEFT_SERIAL --right RIGHT_SERIAL --snapshot calibration
```

如 SDK 安装布局不同，传入 `-DMVS_INCLUDE_DIR=...` 和 `-DMVS_LIBRARY=...`。运行时需让系统动态加载器能找到 MVS 库及其依赖。`--list` 输出序列号、接口和型号。左右相机按序列号绑定，`--snapshot` 会保存一对原尺寸 PPM 图用于取点。

把棋盘格或其他平面标靶放在两台相机的重叠区域。在两张原图上选出相同的四个物理点，尽量分布到重叠区四周。保存为 `site.calib`，格式如下。首行是拼接输出宽、高、左图在输出画布中的 X/Y 原点，后四行是 `左图X 左图Y 右图X 右图Y`。以下数字来自模拟相机，实机要按现场重新测量。输出宽高取偶数，便于编码为 YUV 4:2:0。

```text
1152 352 0 0
520 24 8 32
620 24 108 32
520 320 8 328
620 320 108 328
```

按实测输出尺寸和采集帧率修改下面的 `-video_size` 和 `-framerate`。`--frames 300` 录制 300 张成功配对的拼接帧。去掉该选项后可持续运行，`Ctrl+C` 停止。只想实时预览时，可把 FFmpeg 换成 FFplay 并保留相同的 rawvideo 输入参数。

```sh
set -o pipefail
./build-mvs/dual-hik-stitch --left LEFT_SERIAL --right RIGHT_SERIAL \
  --calibration site.calib --frames 300 |
  ffmpeg -f rawvideo -pixel_format bgr24 -video_size 1152x352 -framerate 30 \
    -i pipe:0 -an -c:v libx264 -pix_fmt yuv420p stitched.mp4
```

RK3588 的 FFmpeg 若包含 `h264_rkmpp`，可把编码参数换成 `-vf format=nv12 -c:v h264_rkmpp -b:v 4M`。先用 `ffmpeg -encoders` 核对编码器。编码与拼接是两段独立处理，当前几何拼接在 CPU 上运行。[Rockchip FFmpeg 编码器文档](https://github.com/nyanmisaka/ffmpeg-rockchip/wiki/Encoder)

## 同步与验收

运动目标跨接缝时，可给两台相机接同一个触发信号，并传 `--trigger Line0`。触发线及 `FrameStart` 节点需按相机型号确认。当前程序仍以主机收帧时间配对，同一触发边沿是否匹配、实际曝光偏差、GigE 传输抖动和拼接帧率都需要实测。标准输出是 BGR24 裸流，FFmpeg 按指定固定帧率生成时间戳。若采集掉帧，录像播放时长会与采集时长不同。

实机先验证两张标定图的公共直线能接上，再用移动标靶穿过接缝，检查错位和重影。记录板端配对帧率、两类丢帧计数和 CPU 占用。连续 10 秒没有配对帧时程序退出并提示排查相机、触发和允许时差。
