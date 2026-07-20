# Runtime 容器环境变量 — SSH 登录不继承 Docker ENV

## 现象

SSH 进狗的 runtime 容器后执行 `ros2 launch`，报错：

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
Caught multiple exceptions when trying to load file of format [py]:
 - KeyError: 'need_compile'
 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown
```

## 原因

仓库中 16 个 launch 文件以硬取值方式读取环境变量：

```python
compiled = os.environ['need_compile']   # 变量不存在直接 KeyError
```

这些变量在 Dockerfile 中通过 `ENV` 定义（`Dockerfile:251-255`）：

| 变量 | 默认值 |
|---|---|
| `need_compile` | `True` |
| `HOST` | `localhost` |
| `MASTER` | `/` |
| `DEPTH_CAMERA_TYPE` | `Dabai` |
| `MACHINE_TYPE` | `JetRover_Acker` |

**Docker 的 `ENV` 只注入容器主进程（PID 1，即 entrypoint / dogbot 服务）**。
通过 `sshd` 登录得到的 shell 是全新会话，不继承这些变量，因此：

- 开机自启的 dogbot 服务运行正常（从 entrypoint 继承 ENV）
- SSH 登录后手动 `ros2 launch` 缺变量 → `KeyError: 'need_compile'`

## 解决

### 长期方案

`.script/template/env_setup.zsh` / `env_setup.bash`（容器内 shell 每次登录都会 source）中加入带默认值的导出，已有值不覆盖：

```bash
export need_compile="${need_compile:-True}"
export HOST="${HOST:-localhost}"
export MASTER="${MASTER:-/}"
export DEPTH_CAMERA_TYPE="${DEPTH_CAMERA_TYPE:-Dabai}"
export MACHINE_TYPE="${MACHINE_TYPE:-JetRover_Acker}"
```

改完模板后需 rebuild runtime 镜像并重新部署到狗（模板在构建时 COPY 进镜像）。

> dev 容器与 runtime 共用同一模板。dev 容器内 `need_compile=True` 会让 launch
> 文件走 `get_package_share_directory()` 分支，行为正确，无副作用。

### 临时热修（不重建镜像）

SSH 进狗的容器，把上面几行直接追加到 `/root/env_setup.zsh` 和
`/root/env_setup.bash`，重新登录即生效。

## 背景：need_compile 的作用

launch 文件用它决定资源路径来源：

```python
if os.environ['need_compile'] == 'True':
    path = get_package_share_directory('peripherals')   # 从 install 空间取
else:
    path = '/home/ubuntu/ros2_ws/src/peripherals'        # 厂商原始镜像的硬编码路径
```

DogBot 部署始终使用 install 空间（`/dogbot_install`），因此该值应恒为 `True`。
