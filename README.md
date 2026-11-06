# RK3588 双海康工业相机视频拼接

两台固定安装、有重叠视野的海康机器人面阵相机分别采集左右画面。程序配对双路帧，用单应矩阵把右图映射到左图坐标系，对重叠区域做加权融合，输出一条视频流。适合从固定平面观察产线、传送带或工装。场景深度差越大，越容易在接缝看到视差。

`PairStream` 和 `StitchStream` 使用 lystream 的 `Stream` / `StreamControl`，可在无 CUDA 的机器上构建。每路相机独占一个采集线程，队列各最多保留四帧。默认按主机收帧时间配对，超过 15 ms 时丢弃较旧帧。每 5 秒输出配对数、运行时长、最大配对时间差、拼接与写出合计最长耗时，以及各路收帧、坏帧、帧号缺口、队列溢出、超差丢帧和重连次数。[lystream 接口源码](https://github.com/PT233/lystream/tree/7f12d207a7514863ac6d225c504a83409d17882c)

## 模拟双摄

[查看模拟拼接视频](demo/stitched.mp4)。成片为 1152 × 352、30 fps、4 秒，包含一条 H.264 视频轨。

需要 CMake 3.18+、Git 和 C++17 编译器。构建会拉取固定版本的 lystream 接口，也可用 `-DLYSTREAM_SOURCE_DIR=/path/to/lystream` 指定已有检出。Windows 若使用的 Ninja 不支持中文路径，可把源码和构建目录放在纯 ASCII 路径下。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
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

如 SDK 安装布局不同，传入 `-DMVS_INCLUDE_DIR=...` 和 `-DMVS_LIBRARY=...`。运行时需让系统动态加载器能找到 MVS 库及其依赖。`--list` 输出序列号、接口和型号。左右相机按序列号绑定，`--snapshot` 会保存一对原尺寸 PPM 图用于取点。可用 `--left-profile` 和 `--right-profile` 指定对应相机导出的 MVS 参数文件，首次打开和断线重连时都会重新加载，再应用命令行指定的触发设置。

把棋盘格或其他平面标靶放在两台相机的重叠区域。在两张原图上选出相同的四个物理点，尽量分布到重叠区四周。保存为 `site.calib`，格式如下。首行是拼接输出宽、高、左图在输出画布中的 X/Y 原点，后四行是 `左图X 左图Y 右图X 右图Y`。以下数字来自模拟相机，实机要按现场重新测量。输出宽高取偶数，便于编码为 YUV 4:2:0。

```text
1152 352 0 0
520 24 8 32
620 24 108 32
520 320 8 328
620 320 108 328
```

正式部署在最后加一行 `cameras LEFT_SERIAL LEFT_WIDTH LEFT_HEIGHT RIGHT_SERIAL RIGHT_WIDTH RIGHT_HEIGHT`，把标定绑定到相机序列号及原始图像尺寸。部署脚本会要求这行。调整相机 ROI、镜头或机位后要复核标定，同尺寸的 ROI 偏移不会自动识别。

按实测输出尺寸和采集帧率修改下面的 `-video_size` 和 `-framerate`。`--frames 300` 录制 300 张成功配对的拼接帧。去掉该选项后可持续运行，`Ctrl+C` 停止。只想实时预览时，可把 FFmpeg 换成 FFplay 并保留相同的 rawvideo 输入参数。

```sh
set -o pipefail
./build-mvs/dual-hik-stitch --left LEFT_SERIAL --right RIGHT_SERIAL \
  --calibration site.calib --frames 300 |
  ffmpeg -f rawvideo -pixel_format bgr24 -video_size 1152x352 -framerate 30 \
    -i pipe:0 -an -c:v libx264 -pix_fmt yuv420p stitched.mp4
```

RK3588 的 FFmpeg 若包含 `h264_rkmpp`，可把编码参数换成 `-vf format=nv12 -c:v h264_rkmpp -b:v 4M`。先用 `ffmpeg -encoders` 核对编码器。几何拼接在 CPU 上运行，构建找到 OpenMP 时可用 `--stitch-threads 2` 或 `4`，实测后选择线程数。[Rockchip FFmpeg 编码器文档](https://github.com/nyanmisaka/ffmpeg-rockchip/wiki/Encoder)

## 曝光、校色与灰度

| 选项 | 行为 |
| --- | --- |
| `--camera-auto-exposure off\|once\|continuous` | 通过 MVS 设置两台相机的 `ExposureAuto`，控制传感器曝光时间。省略时沿用相机或配置文件设置。启用前在 MVS 中确认型号、触发模式及自动曝光上下限。 |
| `--auto-exposure` | 用几何对齐后的重叠区估计亮度比，把右图亮度匹配到左图。 |
| `--auto-color` | 用重叠区估计右图相对左图的 BGR 通道增益，用于匹配两台彩色相机的色差。需要可用的重叠视野，若要测量物体的绝对颜色，需另外做色卡标定。 |
| `--grayscale` | 将拼接结果转为灰度。仍输出三通道 BGR24，三个通道写入相同亮度值，兼容既有 FFmpeg 管道。 |

软件校正每五帧最多取 1024 组对应像素，过滤暗点和接近饱和的采样，用中位数估计增益并限制幅度、平滑更新。有效样本不足时沿用上次结果。建议固定工位先通过 MVS 设置两路相机的曝光和白平衡，再开启软件亮度及色差匹配。光照变化较大时，可在相机允许的模式下试用连续自动曝光，复测运动模糊和两路同步。已过曝的细节需要通过采集设置解决。[海康相机曝光模式说明](https://www.hikrobotics.com/en2/Hikrobotics/Machine%20Vision/01%20Product/%E5%B7%A5%E4%B8%9A%E9%9D%A2%E9%98%B5%E7%9B%B8%E6%9C%BA/CS/%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C/UD26223B_GigE%20Area%20Scan%20Camera%20User%20Manual_V3.4.50_20211117.pdf)

RTSP 服务通过 `CAMERA_AUTO_EXPOSURE`、`AUTO_EXPOSURE`、`AUTO_COLOR` 和 `GRAYSCALE` 启用上述选项。

## Linux 服务部署

将含 MVS 的构建用 `sudo cmake --install build-mvs --prefix /usr/local` 安装。创建 `dualhik` 服务账户，按板厂和海康 SDK 的要求为该账户配置相机与编码设备访问权限。将 `deploy/dual-hik-stitch.service` 安装到 `/etc/systemd/system/`，将 `deploy/service.env.example` 复制为 `/etc/dual-hik-stitch/service.env` 并填写双摄序列号、现场标定和 RTSP 发布地址。RTSP 接收服务须事先启动，例如同板上的 MediaMTX。需要定制安装路径时同步修改服务文件和 `CAPTURE_BIN`。完成后可用 `sudo systemctl daemon-reload`、`sudo systemctl enable --now dual-hik-stitch` 和 `journalctl -u dual-hik-stitch -f` 部署和查看状态。

相机临时中断时，程序清空两路待配对帧并按 250 ms 到 4 秒的退避间隔重试。长时间无法成对、Linux 管道超时或编码进程退出时，由 systemd 重启采集和推流。服务示例中分别配置单路无帧、双路无配对和写入超时。硬触发间隔较长时，要把两项采集超时设得比现场最大间隔长。RTSP 的时间戳使用编码端接收时钟，适用于实时预览。用 CLI 输出 BGR 文件时，FFmpeg 仍按指定的固定帧率生成时间戳。两种输出都需另行验证业务要求的精确曝光时间追溯。

## 同步与验收

运动目标跨接缝时，可给两台相机接同一个触发信号，并传 `--trigger Line0`。触发线及 `FrameStart` 节点需按相机型号确认。支持 `GevIEEE1588` 和 `GevIEEE1588Status` 的 GigE 相机可在现场配置并验证同一 PTP 时钟域，再传 `--device-clock-hz` 和 `--max-skew-us`。程序会核对两台相机的 tick 频率，定期检查 PTP 状态，使用设备时间戳配对。`Master`/`Slave` 状态用于检查相机是否进入工作状态，同一时钟域及实际曝光偏差需用设备诊断和标靶复核。配对窗口应小于半个触发周期，并满足现场误差要求。主机时间模式用于调试或允许更大时差的场景。[海康 GigE 相机手册](https://www.hikrobotics.com/en2/source/vision/document/2023/12/19/UD34944B_GigE%20Area%20Scan%20Camera%20User%20Manual_V4.0.0_20231214.pdf)

按以下顺序验收实板，记录板型、BSP、MVS 版本、相机型号、ROI、像素格式、帧率和网口拓扑。

1. 分别跑单路采集，再跑双路采集。用移动标靶穿过接缝，测对齐误差、曝光偏差、配对帧率和掉帧。切换到目标编码配置后复测端到端延迟、CPU、内存和温度。
2. 拔插单台相机、停止 RTSP 接收端、暂停编码进程并重启板卡。逐项记录恢复时间，检查恢复后的首对帧、丢帧计数和日志，确认恢复前后的帧没有被配到同一对。
3. 在目标工况长跑，例如 72 小时。根据现场事先确定的帧率、延迟、接缝误差和恢复时间阈值出验收报告。当前仓库的自动测试覆盖模拟配对、恢复、输出和并行拼接，真机采集与硬编码仍需要上述验收。

## 双摄自检、逐帧追溯与基线

自检 `--check-pairs N` 实际读取、配对和拼接 N 对图像。相机帧号必须向前推进，源格式和图像尺寸保持一致，标定必须可用。期间发生重连、总超时或采集未完成均返回非零。自检只在标准输出写一条 JSON 结果。真实相机自检会独占相机，请在服务启动前或停服窗口运行。

可通过 `--trace-dir NEW_DIRECTORY` 为有限帧数的自检或普通输出生成 `frames.jsonl`。每行对应一对图像，记录左右帧号、设备原始 tick、SDK 源像素格式、配对所用时钟域及时间差、重连代号、图像从主机收帧到被取用的年龄、拼接耗时和输出写入耗时。自检未写出视频，`write_us` 为 `null`。这些记录能定位采集和拼接，RTSP 解码检查是独立测量，当前无法将接收端的一帧反查到某对源帧。

`tools/board_baseline.py` 需要 Python 3 标准库。用新目录存放证据，工具会保留相机日志、逐帧记录、标定输入与哈希、运行结果和 P50/P95/P99。Linux 上还会按秒记录可读取的进程 CPU 时间、RSS 和板端温度。失败也会留下结果和已产生的记录。

```sh
# 无实物时验证软件流程，报告标明 synthetic。
python3 tools/board_baseline.py capture --app ./build/dual-hik-stitch \
  --calibration config/synthetic.calib --out baseline-synthetic \
  --frames 90 --warmup 5 -- --synthetic

# 将序列号、标定和同步选项换成实际工位值。
python3 tools/board_baseline.py capture --app ./build-mvs/dual-hik-stitch \
  --calibration site.calib --out baseline-camera --frames 90 --warmup 5 -- \
  --left LEFT_SERIAL --right RIGHT_SERIAL --require-bound-calibration

# 服务已运行时，从接收端验证能连续解码指定数量的视频帧。
python3 tools/board_baseline.py rtsp --url rtsp://127.0.0.1:8554/stitch \
  --out baseline-rtsp --frames 90 --timeout 20
```

自检报告中的收帧年龄从主机取到帧开始计时，配对时差也不等于两台相机的光学曝光偏差。RTSP 报告的耗时包含连接和解码，不是端到端图龄。对照性能时使用相同配置及帧数，保留预热帧的记录，并在结论里注明测量时钟和窗口。
