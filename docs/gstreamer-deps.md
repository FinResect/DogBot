# GStreamer 依赖清单 — RTSP Camera Streaming

DogBot 新增 `CameraNode` + `RtspStreamNode` composable node，需要用 GStreamer RTSP Server 将 `/image_raw` 重新编码为 H.264 后通过 RTSP 推流。

GStreamer pipeline: `appsrc → videoconvert → x264enc → rtph264pay`

---

## 1. 开发机（Ubuntu 24.04 x86_64）本地安装

```bash
sudo apt-get update

# 编译依赖（头文件 + pkg-config .pc）
sudo apt-get install -y \
    libgstrtspserver-1.0-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

# 运行时依赖（插件 .so）
sudo apt-get install -y \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools
```

验证：

```bash
pkg-config --modversion gstreamer-rtsp-server-1.0   # 应输出类似 1.20.3
pkg-config --modversion gstreamer-1.0
gst-inspect-1.0 x264enc    # 编码器
gst-inspect-1.0 rtph264pay # RTP 打包器
```

---

## 2. Dockerfile — 运行时镜像 (dogbot-base, Ubuntu 22.04)

在 `dogbot-base` 阶段的 `apt-get install` 追加以下包：

```dockerfile
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-ugly \
gstreamer1.0-tools
```

> `libgstreamer1.0-0`、`libgstreamer-plugins-base1.0-0`、`libgstrtspserver-1.0-0` 会作为依赖自动安装。

---

## 3. Dockerfile — 交叉编译 sysroot (dogbot-sysroot-arm64, Ubuntu 22.04 ARM64)

在 `dogbot-sysroot-arm64` 阶段的 `apt-get install` 追加：

```dockerfile
libgstrtspserver-1.0-dev \
libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev
```

这些 `-dev` 包提供 ARM64 的 `.so`、`.pc` 和头文件，供交叉编译时链接。

---

## 4. Dockerfile — 开发容器 (dogbot-develop)

如需在 dev 容器中**原生编译**（非交叉编译），在 `dogbot-develop` 阶段同样追加第 1 节的编译依赖。

---

## 5. RTSP 测试

容器内：

```bash
source /dogbot_install/local_setup.bash
ros2 launch dogbot_start start.launch.py
```

宿主机拉流（容器用 `--network=host`，直接访问 localhost）：

```bash
ffplay rtsp://localhost:8554/cam
# 或
vlc rtsp://localhost:8554/cam
```

在浏览器中用 MJPEG 查看（需额外启动 web_video_server 或等价 HTTP bridge）。
