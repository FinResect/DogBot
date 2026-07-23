# DogBot AGENTS.md

## 构建（Build）

所有构建命令在 `.script/` 中，从仓库根目录执行。

```bash
# amd64 本地构建（colcon + Ninja），透传任意 colcon 参数：
./.script/build-dogbot [colcon args...]
# 例：单包构建：
./.script/build-dogbot --packages-select dogbot_core

# arm64 交叉编译（目标树莓派 5）：
./.script/build-dogbot-cross [colcon args...]

# 清理所有构建产物（本地 + 交叉）：
./.script/clean-dogbot
```

脚本自动 unset 所有 ROS/CMake/PKG_CONFIG 环境变量再 source `/opt/ros/humble/setup.bash`。`DOGBOT_PATH` 从脚本位置推断，也可通过环境变量覆盖。

```bash
# 本地启动：
./.script/launch-dogbot   # source install/setup.bash，然后 ros2 launch dogbot_start start.launch.py
```

## 测试（Testing）

仅有 ROS2 ament lint 检查，无单元/功能测试。通过 CMake `BUILD_TESTING` 开关控制。

```bash
# 启用测试并运行全部：
./.script/build-dogbot --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install --return-code-on-test-failure

# 单包测试：
colcon test --merge-install --packages-select dogbot_core --return-code-on-test-failure

# 单条 Python lint 测试：
colcon test --merge-install --packages-select large_models --pytest-args -k test_flake8
```

Python 包三个 linter：`test_flake8.py`、`test_pep257.py`、`test_copyright.py`，使用 `@pytest.mark.flake8` / `@pytest.mark.linter`。C++ 包使用 `ament_lint_auto`，无实际 C++ 单元测试。

## 功能包布局（Package layout）

14 个包位于 `ros2_ws/src/`：

- **C++（ament_cmake）**：`dogbot_core`、`dogbot_start`、`interfaces`、`serial_bridge`、`large_models_msgs`、`ros_robot_controller_msgs`、`puppy_control_msgs`
- **Python（ament_python）**：`app`、`large_models`、`lab_config`、`peripherals`、`slam`、`navigation`、`driver/*`、`simulations/*`、`example/*`

所有 Python 包使用统一 `setup.cfg`：
```ini
[develop]
script_dir=$base/lib/<package_name>
[install]
install_scripts=$base/lib/<package_name>
```

## 架构约定（Architecture boundaries）

硬件驱动、运动学解算、轨迹规划分为三层，通过 ROS2 topic 解耦：

```
┌──────────────────────────────────────────────────┐
│  LegController (控制层)                           │
│  轨迹规划 + 步态生成 → 调用 LegSolver IK          │
│  pub: left_front_{hip,knee}/control_angle         │
│  sub: foot_target (geometry_msgs::Point)           │
└──────────────────────┬───────────────────────────┘
                       │ std_msgs::Float64 (degrees)
                       ▼
┌──────────────────────────────────────────────────┐
│  DogBot (硬件层)                                  │
│  ZX30S 舵机 ×8 → 20Hz 定时器 → /dev/ttyAMA0       │
│  sub: <prefix>/control_angle                      │
└──────────────────────────────────────────────────┘
```

| 文件 | 路径 | 职责 |
|---|---|---|
| `dogbot.cpp` | `dogbot_core/src/hardware/` | 仅做外设管理：串口初始化、舵机通信、20Hz PWM 定时发送 |
| `leg_solver.hpp` | `dogbot_core/src/controller/` | 纯运动学解算：输入足端坐标，输出髋/膝关节角度 |
| `leg_controller.cpp` | `dogbot_core/src/controller/` | 足端轨迹规划 + 步态生成：按不同轨迹方程计算足端轨迹，经 IK 映射后发布 topic |

### 步态与轨迹

- `leg_controller.cpp` 中应包含不同轨迹方程来控制不同步态（Trot、Amble、Walk 等）
- 不同 gait 使用对应 `.hpp` 类组织，每个步态类输出足端轨迹供 IK 消费
- Python 旧版参考：`driver/puppy_control/puppy_control/puppy.py` 已有 Trot/Amble/Walk 实现

### 交互方式

DogBot 和 LegController 在代码层面完全解耦，仅通过 ROS2 topic 通信：
- `ZX30S`（DogBot 内部）订阅 `"<prefix>/control_angle"`（`std_msgs::Float64`），收到后转为 PWM
- DogBot 的 20Hz 定时循环统一将所有舵机的 `target_pwm_` 通过串口发出
- LegController 发布角度到 `left_front_hip/control_angle` 和 `left_front_knee/control_angle`
- 两者编译进同一共享库（`dogbot_core_lib`），注册为 composable node

## 交叉编译（Cross-compilation）

目标 `aarch64-linux-gnu`（arm64）。要求：

1. Sysroot 位于 `/opt/sysroots/arm64`（从 `dogbot-base:arm64` 镜像提取）
2. 工具链配置：`ros2_ws/toolchain.cmake`
3. 编译器：`aarch64-linux-gnu-gcc-12` / `aarch64-linux-gnu-g++-12`

编译产物输出到 `build-cross/`、`install-cross/`、`log-cross/`（与本地构建独立）。构建 sysroot 镜像后运行 `./.script/fix-sysroot-cross-paths` 修复 sysroot 中的绝对路径。

## 启动系统（Launch system）

`dogbot_start` 是顶层启动编排器。新增包节点步骤：

1. 将包名加入 `dogbot_start/launch/start.launch.py` 的 `PACKAGES` 元组
2. 在包内创建 `config/nodes.yaml`，`executable` 与 `CMakeLists.txt` 中 `add_executable()` 名称一致

```yaml
# config/nodes.yaml
nodes:
  - executable: my_node
  - executable: another_node
```

## 环境变量（Environment variables）

| 变量 | 默认值 | 作用 |
|---|---|---|
| `need_compile` | `True` | 控制 launch 中资源路径来源（硬编码 `os.environ`，缺失触发 KeyError） |
| `HOST` | `localhost` | 目标主机 |
| `MASTER` | `/` | ROS2 主节点 |
| `DEPTH_CAMERA_TYPE` | `Dabai` | 深度相机型号 |
| `MACHINE_TYPE` | `JetRover_Acker` | 底盘型号 |

树莓派运行时容器中 SSH 登录不继承 Docker `ENV`，变量通过 `.script/template/env_setup.zsh` / `env_setup.bash` 重新导出。详见 `docs/runtime-env-vars.md`。

## Docker

5 个多阶段构建 target。构建顺序不能乱：

```bash
# 1. Base（arm64，提供 sysroot）
docker buildx build . --platform linux/arm64 --target dogbot-base -t dogbot-base:arm64 --load
# 2. Develop-full（amd64，消费 base）
docker buildx build . --platform linux/amd64 --target dogbot-develop-full \
  --build-arg SYSROOT_IMAGE_ARM64=dogbot-base:arm64 --load
```

Dev Container 配置见 `.devcontainer/devcontainer.json`。容器需 `--privileged` 和 `--net=host`。

## CI

唯一工作流：`.github/workflows/update-image.yml`。仅 `main` 分支触发（条件：Dockerfile、`.script/build-dogbot`、`ros2_ws/toolchain.cmake`、工作流本身变更）。支持 `workflow_dispatch` 手动触发。CI 验证步骤做交叉编译冒烟测试 + `readelf -h` 确认 AArch64 架构。

## 同步与部署（Sync / Deploy）

- `.script/sync-remote` — Unison 持续同步 `install-cross/` → `ssh://remote//dogbot_install`
- `.script/wait-sync` — 阻塞直到一轮同步完成
- `.script/set-remote` — 交互式 mDNS 发现 + SSH 配置

部署：`rsync install-cross-arm64/` → Pi 的 `/opt/dogbot/install/`，然后 `docker restart dogbot`。

## 代码风格与格式化

- C++：`.clang-format`（LLVM 风格，ColumnLimit 100，IndentWidth 4，C++20）。clang-tidy 通过 clangd LSP 运行。
- Python：flake8、pep257、ament_copyright（以 colcon test 方式运行）。
- 无 pre-commit hooks。无 Makefile。

## 关键文档

- `ENVIRONMENT.md` — 项目概览、架构、容器镜像
- `docs/zh-cn/workflow.md` — 完整开发工作流
- `docs/zh-cn/deploy-runtime.md` — 树莓派 5 部署指南
- `docs/zh-cn/launch-system.md` — 启动系统架构与新增包说明
- `docs/runtime-env-vars.md` — SSH 环境变量继承问题
