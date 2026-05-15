# 使用官方桌面版作为基础镜像
# FROM osrf/ros:humble-desktop-full
FROM osrf/ros:jazzy-desktop-full

# 设置环境变量
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
# 显式指定 Gazebo 版本
ENV GZ_VERSION=harmonic

# 切换至 root 进行安装
USER root
USER root
# 同时尝试修改 sources.list 和新的 ubuntu.sources
RUN if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then \
        sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources && \
        sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources; \
    fi && \
    if [ -f /etc/apt/sources.list ]; then \
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
    git \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# install gazebo
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install -y \
#     ros-humble-ros-gz-sim \
#     ros-humble-ros-gz-bridge \
#     ros-humble-ros-gz-interfaces \
#     ros-humble-image-transport \
#     ros-humble-xacro \
#     ros-humble-robot-state-publisher \
#     ros-humble-joint-state-publisher \
#     ros-humble-gz-ros2-control \
#     && rm -rf /var/lib/apt/lists/*

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN git config --global http.postBuffer 524288000 && \
    git config --global core.compression 0 && \
    if [ ! -d "$HOME/.oh-my-zsh" ]; then \
    sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" "" --unattended; \
    fi && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo '# Hint: uncomment and set DOGBOT_PATH if DOGBOT is not located at /workspaces/DogBot.' >> ~/.zshrc
    # echo '# export DOGBOT_PATH="/workspaces/DogBot"' >> ~/.zshrc && \
    # echo 'source ~/env_setup.zsh' >> ~/.zshrc
    # echo 'source /opt/ros/jazzy/setup.zsh' >> ~/.zshrc

RUN chsh -s $(which zsh)
# 自动 source ROS 2 环境
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/jazzy/setup.zsh" >> ~/.zshrc && \
    echo 'export DOGBOT_PATH="/workspaces/DogBot"' >> ~/.zshrc && \
    echo 'export PATH="/workspaces/DogBot/.script:${PATH}"' >> ~/.zshrc


ENV PATH="/workspaces/DogBot/.script:${PATH}"

CMD ["bash"]