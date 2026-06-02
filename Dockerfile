# 1. 修改基础镜像为 Humble 官方桌面满血版
FROM osrf/ros:humble-desktop-full

# 设置环境变量
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
# Humble 对应的 Gazebo 版本通常是 Fortress（默认）或 Isaac Sim 等，这里显式指定
ENV GZ_VERSION=fortress

# 切换至 root 进行安装
USER root

# 2. 简化换源脚本（Ubuntu 22.04 只需要修改标准的 sources.list）
RUN if [ -f /etc/apt/sources.list ]; then \
        sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
        sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list; \
    fi

# 安装常用工具和 ROS 2 编译工具
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
    iputils-ping \
    zsh \
    wget \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 3. 将 Gazebo 和相关 ROS 包更改为 humble 版本
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN git config --global http.postBuffer 524288000 && \
    git config --global core.compression 0 && \
    if [ ! -d "$HOME/.oh-my-zsh" ]; then \
    sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" "" --unattended; \
    fi && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo '# Hint: uncomment and set DOGBOT_PATH if DOGBOT is not located at /workspaces/DogBot.' >> ~/.zshrc

RUN chsh -s $(which zsh)

# 4. 自动 source ROS 2 环境修改为 humble
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc && \
    echo 'export DOGBOT_PATH="/workspaces/DogBot"' >> ~/.zshrc && \
    echo 'export PATH="/workspaces/DogBot/.script:${PATH}"' >> ~/.zshrc

ENV PATH="/workspaces/DogBot/.script:${PATH}"

# 5. 创建 XDG_RUNTIME_DIR 并赋予正确权限，解决 vscode-git.sock 无法创建的问题
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

CMD ["/entrypoint"]