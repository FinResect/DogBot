# 树莓派 5 部署指南

## 首次部署

### 1. 同步编译产物到 Pi

在 x86 宿主机上（dev 容器外）：

```bash
rsync -avz ros2_ws/install-cross-arm64/ pi@<树莓派IP>:/opt/dogbot/install/
```

### 2. 加载镜像

```bash
# 方式A：拉取镜像（GitHub Action 已推送至 Docker Hub）
docker pull dogbot-runtime:arm64

# 方式B：从本地 tar 加载
docker load -i dogbot-runtime-arm64.tar
```

### 3. 启动容器

```bash
docker run -d --restart=always --privileged --network=host \
  -v /dev:/dev \
  -v /opt/dogbot/install:/dogbot_install \
  --name dogbot \
  dogbot-runtime:arm64
```

| 参数 | 作用 |
|---|---|
| `-d` | 后台运行 |
| `--restart=always` | 崩溃自动重启 + 开机自启 |
| `--privileged` | 容器拥有宿主机所有硬件访问权限 |
| `--network=host` | 共享宿主机网络栈（ROS2 发现需要） |
| `-v /dev:/dev` | 把宿主机全部 `/dev` 挂进容器，免去逐个 `--device` |
| `-v /opt/dogbot/install:/dogbot_install:ro` | 交叉编译产物只读挂载 |
| `--name dogbot` | 容器名 |

> 容器内域名解析失败时可追加 `--dns <路由器IP> --dns 8.8.8.8`。

### 4. 查看运行状态

```bash
docker logs -f dogbot           # 实时日志
docker logs --tail 50 dogbot    # 最近 50 行
docker exec dogbot screen -ls   # 查看 screen 会话
docker exec -it dogbot screen -r dogbot  # 进入 ROS2 终端
docker exec -it dogbot zsh      # 进入容器 zsh
```

---

## 更新部署（代码变更后）

### 1. 同步新的编译产物

```bash
rsync -avz ros2_ws/install-cross-arm64/ pi@<树莓派IP>:/opt/dogbot/install/
```

### 2. 重启容器

```bash
ssh pi@<树莓派IP> "docker restart dogbot"
```

---

## 停止 / 删除容器

```bash
docker stop dogbot    # 停止
docker rm dogbot      # 删除（保留镜像和 /opt/dogbot/install）
```

---

## 排查

```bash
# 看容器是否在跑
docker ps | grep dogbot

# 看容器内 ROS2 节点
docker exec dogbot bash -c "source /dogbot_install/local_setup.bash && ros2 node list"

# 看容器内串口设备
docker exec dogbot ls /dev/tty*

# 进入 screen 看 ROS 输出
docker exec -it dogbot screen -r dogbot
# 在 screen 内按 Ctrl+A 然后按 D 断开
```
