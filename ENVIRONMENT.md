# DogBot 开发环境概述

## 整体架构

```
┌─────────────────────────────────────────────────┐
│                   开发机 (amd64)                  │
│  ┌───────────────────────────────────────────┐  │
│  │          Dev Container (dogbot-develop)     │  │
│  │  - ROS2 Humble                             │  │
│  │  - 交叉编译工具链 (aarch64)                 │  │
│  │  - LSP / 调试 / 开发工具                    │  │
│  │  - 通过 Unison 同步代码到树莓派             │  │
│  └───────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
                        │
          SSH / Unison 同步
                        │
                        ▼
┌─────────────────────────────────────────────────┐
│              树莓派 5 (arm64)                     │
│  ┌───────────────────────────────────────────┐  │
│  │        Runtime Container (dogbot-runtime)  │  │
│  │  - ROS2 Humble (运行时仅需基础依赖)         │  │
│  │  - dogbot 服务自启动 (通过 tini)            │  │
│  │  - SSH 端口 2022                            │  │
│  │  - Avahi mDNS 服务发现                      │  │
│  │  - 安装路径: /dogbot_install/               │  │
│  └───────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
```

## 容器镜像

项目通过单个 `Dockerfile` 多阶段构建定义三种镜像：

| 阶段 (Target) | 平台 | 用途 |
|---|---|---|
| `dogbot-develop` | amd64 | 开发容器（代码编写、构建） |
| `dogbot-develop-full` | amd64 | 开发容器 + 交叉编译链（aarch64 交叉编译） |
| `dogbot-runtime` | arm64 | 树莓派 5 运行时容器（部署运行） |
| `dogbot-sysroot-arm64` | arm64 | ARM64 系统根文件系统（供交叉编译使用） |
| `dogbot-base` | 多平台 | 基础镜像，包含 ROS2 及通用依赖 |

## ROS2 环境

- **发行版**: ROS2 Humble
- **工作空间**: `ros2_ws/`

### 功能包列表 (`ros2_ws/src/`)

| 包名 | 说明 |
|---|---|
| `app` | 应用层（上层业务逻辑） |
| `dogbot_core` | 核心模块 |
| `dogbot_start` | 启动管理 |
| `driver` | 硬件驱动 |
| `example` | 示例代码 |
| `interfaces` | 自定义 ROS2 消息/服务接口 |
| `lab_config` | 实验室配置 |
| `large_models` | 大模型相关 |
| `large_models_msgs` | 大模型消息定义 |
| `navigation` | 导航功能 |
| `peripherals` | 外设驱动（传感器、执行器等） |
| `serial_bridge` | 串口通信桥 |
| `simulations` | 仿真相关 |
| `slam` | SLAM 定位建图 |

### 工具链

- **编译器**: GCC 12 / Clang 18
- **构建系统**: CMake 4.2 + Ninja + colcon
- **LSP**: clangd
- **格式化**: clang-format, clang-tidy
- **调试器**: lldb

### 交叉编译

- 开发容器(amd64) 交叉编译 arm64 目标，通过 `dogbot-sysroot-arm64` 提供 ARM64 系统根文件系统
- 交叉编译工具链配置: `ros2_ws/toolchain.cmake`

## 代码同步

- 使用 **Unison** 将开发容器中的代码/构建产物同步到树莓派的运行时容器
- SSH 密钥在开发容器构建时生成，运行时容器继承公钥以实现免密登录

## 关键环境变量

| 变量 | 默认值 | 说明 |
|---|---|---|
| `DOGBOT_PATH` | `/workspaces/DogBot` | 项目根路径 |
| `need_compile` | `True` | 控制 launch 文件资源路径来源 |
| `HOST` | `localhost` | 目标主机 |
| `MASTER` | `/` | ROS2 主节点 |
| `DEPTH_CAMERA_TYPE` | `Dabai` | 深度相机型号 |
| `MACHINE_TYPE` | `JetRover_Acker` | 机器人底盘型号 |

## 主要依赖

- **计算机视觉**: OpenCV, PCL (点云库)
- **线性代数**: Eigen3, Ceres Solver
- **硬件**: libusb
- **导航**: Nav2 (Navigation2)
- **可视化**: foxglove-bridge
- **深度学习模型部署**: large_models 模块

## 运行时服务

Runtime 容器通过 `tini` 作为 PID 1，启动 `entrypoint` 脚本，该脚本:
1. 启动 SSH 服务 (端口 2022)
2. 启动 Avahi mDNS 服务
3. 启动 dogbot 主服务 (`/etc/init.d/dogbot`)

> **注意**: SSH 登录后的 shell 不继承 Dockerfile 中 `ENV` 定义的环境变量，需由 `env_setup.sh` / `env_setup.zsh` 补充导出。详见 `docs/runtime-env-vars.md`。
