# DogBot 开发与部署工作流

## 架构概览

```
开发机 (amd64)                          机器人/树莓派 (arm64)
┌─────────────────────────┐            ┌──────────────────────────┐
│  dogbot-develop-full     │            │  dogbot-runtime          │
│                          │  unison    │                          │
│  ros2_ws/                │  ───────→  │  /dogbot_install/        │
│   install-cross-arm64/   │   sync     │                          │
│                          │            │  service dogbot start    │
│  aarch64 交叉编译工具链    │            │   -> ros2 launch ...     │
│  /opt/sysroots/arm64     │            │                          │
└─────────────────────────┘            └──────────────────────────┘
```

- 在 amd64 开发机上用 aarch64 交叉编译器构建 arm64 产物
- 通过 Unison 自动同步到树莓派的 `/dogbot_install/`
- 树莓派上的 runtime 容器读取同步过来的产物并启动 ROS 节点

---

## 前置条件

- Docker Engine ≥ 23.0（需 Buildx）
- QEMU 支持（模拟 arm64 构建）：

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

- 树莓派已装 Docker

---

## 一、构建镜像

所有命令在仓库根目录执行。

### 1.1 构建 dogbot-base（arm64）

交叉编译的 sysroot 来源，包含 ROS Humble + 所有依赖库。

```bash
docker buildx build . --platform linux/arm64 --target dogbot-base -t dogbot-base:arm64 --load
```

> 首次 20-40 分钟，GHA cache 会加速后续构建。

### 1.2 构建 dogbot-develop-full（amd64）

开发者镜像，内嵌 arm64 sysroot 和 aarch64 交叉工具链。

```bash
docker buildx build . --platform linux/amd64 --target dogbot-develop-full \
  --build-arg SYSROOT_IMAGE_ARM64=dogbot-base:arm64 --load
```

### 1.3 构建 dogbot-runtime（arm64）

树莓派端运行时镜像。

```bash
docker buildx build . --platform linux/arm64 --target dogbot-runtime -t dogbot-runtime:arm64 --load
```

---

## 二、进入开发容器

### 方式 A：VSCode Dev Containers

`devcontainer.json` 已配置好 `build.target: dogbot-develop-full`，直接用 VSCode 的 "Reopen in Container"。

### 方式 B：命令行

```bash
docker run -it --rm \
  --net=host --ipc=host --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspaces/DogBot \
  dogbot-develop-full:latest zsh
```

容器内脚本自动可用（`DOGBOT_PATH`、`PATH` 已由 Dockerfile 注入，`env_setup.zsh` 由 `~/.zshrc` source）。

---

## 三、日常开发

### 3.1 交叉编译

```bash
# 完整构建
build-dogbot

# 按需构建
build-dogbot --packages-up-to ros_robot_controller
build-dogbot --packages-select ros_robot_controller_msgs
```

产物：`ros2_ws/install-cross-arm64/`

### 3.2 同步到树莓派

```bash
# 将交叉编译产物软链到 sync-remote 使用的目录
rm -rf ros2_ws/install
ln -s install-cross-arm64 ros2_ws/install

# 启动持续同步（文件变更自动推送）
sync-remote

# 等待一次完整同步完成（适合 CI/脚本）
wait-sync
```

`sync-remote` 用 Unison 监听 `ros2_ws/install/`，变更自动推送到 `ssh://remote//dogbot_install`。

### 3.3 清理

```bash
# 清理 native 构建目录
clean-dogbot

# 清理交叉编译目录
rm -rf ros2_ws/build-cross-arm64 ros2_ws/install-cross-arm64 ros2_ws/log-cross-arm64
```

---

## 四、部署到树莓派

### 4.1 导出镜像

```bash
docker save dogbot-runtime:arm64 | gzip > dogbot-runtime.tar.gz
```

### 4.2 传到树莓派

```bash
scp dogbot-runtime.tar.gz remote:~
```

`remote` 由 `set-remote` 配置（见 4.3）。

### 4.3 配置 SSH（首次）

```bash
./.script/set-remote
```

按交互提示配置树莓派 IP、用户名。完成后 `~/.ssh/config` 会新增 `remote` Host。

也可手动编辑：

```plain
Host remote
    HostName <树莓派 IP>
    Port 2022
    User root
    PreferredAuthentications publickey
    IdentityFile ~/.ssh/id_rsa
```

### 4.4 在树莓派上加载并启动

```bash
# SSH 到树莓派
ssh-remote

# 加载镜像
docker load < dogbot-runtime.tar.gz

# 创建挂载目录
mkdir -p /dogbot_install

# 启动容器
docker run -d --restart unless-stopped \
  --name dogbot \
  --net=host --privileged \
  -v /dogbot_install:/dogbot_install:ro \
  dogbot-runtime:arm64
```

`--privileged` 是为了访问 USB、GPIO 等硬件设备，可根据需求收紧。

---

## 五、树莓派端操作

### 5.1 管理 DogBot 服务

```bash
# 查看状态
service dogbot

# 启动
service dogbot start

# 停止
service dogbot stop

# 重启（会重新读取 /dogbot_install 中的最新代码）
service dogbot restart
```

### 5.2 查看日志

```bash
# 从开发机附加到树莓派上的 screen 会话
attach-remote

# 或者 SSH 后手动
ssh-remote
service dogbot attach
```

### 5.3 常用 SSH 命令

```bash
# 交互式 shell
ssh-remote

# 执行单条命令
ssh-remote "source /dogbot_install/local_setup.bash && ros2 topic list"
```

---

## 六、脚本速查

| 脚本 | 运行位置 | 用途 |
|---|---|---|
| `build-dogbot` | 容器内 | 交叉编译 arm64 → `install-cross-arm64/` |
| `clean-dogbot` | 容器内 | 清理 native build/install/log |
| `sync-remote` | 容器内 | 持续同步 install/ 到树莓派 |
| `wait-sync` | 容器内 | 阻塞直到一次完整同步完成 |
| `set-remote` | 容器内 | 交互式配置 SSH remote Host |
| `launch-dogbot` | 容器内 | source env 后执行 ros2 命令 |
| `ssh-remote` | 容器内 | SSH 到树莓派 |
| `attach-remote` | 容器内 | 附加到树莓派 dogbot screen 会话 |
| `attach-remote -r` | 容器内 | 重启服务并附加 |

---

## 七、CI/CD

推送到 `main` 且变更涉及 Dockerfile/toolchain/build 脚本时自动触发。

流水线：**build_base** → **build_images** → **verify**（交叉编译 + readelf 验证 AArch64）→ **promote**

所需 GitHub Secrets：

| Secret | 用途 |
|---|---|
| `DOCKERHUB_USERNAME` | Docker Hub 用户名 |
| `DOCKERHUB_TOKEN` | Docker Hub access token |
| `CONTAINER_ID_RSA` | 容器 SSH 私钥 |
| `CONTAINER_ID_RSA_PUB` | 容器 SSH 公钥 |

---

## 八、问题排查

| 现象 | 解决 |
|---|---|
| `dogbot-base:arm64: pull access denied` | 先跑 1.1 构建 dogbot-base |
| `sysroot not found` / `cross compiler not found` | 用的是 `dogbot-develop` 而非 `dogbot-develop-full`。重新跑 1.2 |
| `sync-remote` 连不上 | 跑 `set-remote` 更新 IP；确认树莓派 2022 端口可达 |
| 树莓派 `/dogbot_install` 挂载后服务起不来 | 确保目录非空，且至少有一次成功交叉编译 + sync |
| 首次 buildx 很慢 | 正常，QEMU 模拟 arm64 编译无原生加速 |
| 需要代理 | 追加 `--network host --build-arg HTTP_PROXY=... --build-arg HTTPS_PROXY=...` |
