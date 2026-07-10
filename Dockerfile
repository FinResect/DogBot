ARG SYSROOT_IMAGE_AMD64=dogbot-sysroot-amd64:latest
ARG SYSROOT_IMAGE_ARM64=dogbot-sysroot-arm64:latest

FROM ros:humble-ros-base AS dogbot-base
ARG TARGETARCH

SHELL ["/bin/bash", "-c"]

ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    wget curl unzip \
    zsh screen tmux \
    git vim \
    usbutils net-tools iputils-ping \
    ripgrep htop fzf \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libopencv-dev \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-msgs \
    ros-humble-xacro \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-foxglove-bridge \
    && case "${TARGETARCH}" in \
        amd64) apt-get install -y --no-install-recommends \
            libgoogle-glog-dev \
            libgflags-dev \
            libatlas-base-dev \
            libsuitesparse-dev \
            libceres-dev ;; \
         arm64) echo "Skipping amd64-only -dev packages for arm64 runtime" ;; \
    esac \
    && apt-get clean || true && \
    rm -rf /var/lib/apt/lists/* /tmp/* || true

RUN apt-get update && apt-get install -y --no-install-recommends \
    ocaml-nox \
    && curl -L "https://github.com/bcpierce00/unison/archive/refs/tags/v2.53.8.tar.gz" \
        -o "/tmp/unison-v2.53.8.tar.gz" && \
    echo "d0d30ea63e09fc8edf10bd8cbab238fffc8ed510d27741d06b5caa816abd58b6  /tmp/unison-v2.53.8.tar.gz" | sha256sum -c - && \
    tar -xzf "/tmp/unison-v2.53.8.tar.gz" -C /tmp && \
    cd "/tmp/unison-2.53.8" && \
    make -j"$(nproc)" && \
    make install PREFIX=/usr/local && \
    cd / && \
    rm -rf "/tmp/unison-2.53.8" "/tmp/unison-v2.53.8.tar.gz" && \
    apt-get purge -y --auto-remove ocaml-nox && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN rosdep update

FROM dogbot-base AS dogbot-develop
ARG TARGETARCH

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential gcc-12 g++-12 \
    cmake ninja-build \
    openssh-client \
    lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN curl -fsSL https://deb.nodesource.com/setup_24.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

ARG LLVM_VERSION=18
RUN mkdir -p /etc/apt/keyrings && \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /etc/apt/keyrings/apt.llvm.org.gpg && \
    chmod 644 /etc/apt/keyrings/apt.llvm.org.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/apt.llvm.org.gpg] https://apt.llvm.org/jammy/ llvm-toolchain-jammy-${LLVM_VERSION} main" \
    > /etc/apt/sources.list.d/llvm.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libomp-${LLVM_VERSION}-dev \
    clang-${LLVM_VERSION} clangd-${LLVM_VERSION} clang-format-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} \
    lldb-${LLVM_VERSION} lld-${LLVM_VERSION} llvm-${LLVM_VERSION} && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/lldb lldb /usr/bin/lldb-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ar llvm-ar /usr/bin/llvm-ar-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ranlib llvm-ranlib /usr/bin/llvm-ranlib-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/ld.lld ld.lld /usr/bin/ld.lld-${LLVM_VERSION} 50 && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

RUN --mount=type=bind,target=/tmp/.ssh,source=.ssh,readonly=false \
    mkdir -p /home/ubuntu/.ssh && \
    if [ ! -f "/tmp/.ssh/id_rsa" ]; then ssh-keygen -N "" -f "/tmp/.ssh/id_rsa"; fi && \
    cp -r /tmp/.ssh/* /home/ubuntu/.ssh && \
    chown -R 1000:1000 /home/ubuntu && chmod 600 /home/ubuntu/.ssh/id_rsa && \
    mkdir -p /home/ubuntu/.unison && \
    echo 'confirmbigdel = false' >> /home/ubuntu/.unison/default.prf && \
    chown -R 1000:1000 /home/ubuntu/.unison

RUN case "${TARGETARCH}" in \
        amd64) nvim_arch=x86_64 ;; \
        arm64) nvim_arch=arm64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    curl -LO "https://github.com/neovim/neovim/releases/latest/download/nvim-linux-${nvim_arch}.tar.gz" && \
    rm -rf /opt/nvim && \
    tar -C /opt -xzf "nvim-linux-${nvim_arch}.tar.gz" && \
    mv "/opt/nvim-linux-${nvim_arch}" /opt/nvim && \
    rm "nvim-linux-${nvim_arch}.tar.gz"
ENV PATH="${PATH}:/opt/nvim/bin"

RUN case "${TARGETARCH}" in \
        amd64) cmake_arch=x86_64 ;; \
        arm64) cmake_arch=aarch64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    wget "https://github.com/kitware/cmake/releases/download/v4.2.3/cmake-4.2.3-linux-${cmake_arch}.sh" -O install.sh && \
    mkdir -p /opt/cmake/ && bash install.sh --skip-license --prefix=/opt/cmake/ --exclude-subdir && \
    rm install.sh
ENV PATH="/opt/cmake/bin:${PATH}"

RUN useradd -m -u 1000 -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p \
        /home/ubuntu/.agents \
        /home/ubuntu/.cache \
        /home/ubuntu/.config \
        /home/ubuntu/.local/share \
        /home/ubuntu/.local/state && \
    chown -R ubuntu:ubuntu /home/ubuntu

WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV HOME=/home/ubuntu
USER ubuntu

RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    sed -i 's/plugins=(git)/plugins=()/g' ~/.zshrc && \
    sed -i "s/# zstyle ':omz:update' mode disabled/zstyle ':omz:update' mode disabled/g" ~/.zshrc && \
    echo '# export DOGBOT_PATH="/workspaces/DogBot"' >> ~/.zshrc

FROM --platform=linux/amd64 ${SYSROOT_IMAGE_AMD64} AS dogbot-sysroot-amd64
FROM --platform=linux/arm64 ${SYSROOT_IMAGE_ARM64} AS dogbot-sysroot-arm64

FROM dogbot-develop AS dogbot-develop-full
ARG TARGETARCH

USER root

RUN apt-get update && \
    case "${TARGETARCH}" in \
        amd64) cross_triplet=aarch64-linux-gnu; cross_pkg_triplet=aarch64-linux-gnu ;; \
        arm64) cross_triplet=x86_64-linux-gnu; cross_pkg_triplet=x86-64-linux-gnu ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    apt-get install -y --no-install-recommends \
        "gcc-12-${cross_pkg_triplet}" "g++-12-${cross_pkg_triplet}" \
        "binutils-${cross_pkg_triplet}" && \
    update-alternatives --install "/usr/bin/${cross_triplet}-gcc" "${cross_triplet}-gcc" "/usr/bin/${cross_triplet}-gcc-12" 50 && \
    update-alternatives --install "/usr/bin/${cross_triplet}-g++" "${cross_triplet}-g++" "/usr/bin/${cross_triplet}-g++-12" 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN mkdir -p /opt/sysroots && \
    case "${TARGETARCH}" in \
        amd64) mkdir -p /opt/sysroots/arm64 ;; \
        arm64) mkdir -p /opt/sysroots/amd64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

RUN --mount=from=dogbot-sysroot-amd64,target=/mnt/sysroot-amd64,readonly \
    --mount=from=dogbot-sysroot-arm64,target=/mnt/sysroot-arm64,readonly \
    set -euo pipefail && \
    case "${TARGETARCH}" in \
        amd64) tar \
            --exclude='./dev/*' \
            --exclude='./proc/*' \
            --exclude='./sys/*' \
            --exclude='./run/*' \
            --exclude='./tmp/*' \
            -C /mnt/sysroot-arm64 -cf - . | tar -C /opt/sysroots/arm64 -xf - ;; \
        arm64) tar \
            --exclude='./dev/*' \
            --exclude='./proc/*' \
            --exclude='./sys/*' \
            --exclude='./run/*' \
            --exclude='./tmp/*' \
            -C /mnt/sysroot-amd64 -cf - . | tar -C /opt/sysroots/amd64 -xf - ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV HOME=/home/ubuntu
USER ubuntu

FROM dogbot-base AS dogbot-runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    tini \
    openssh-server \
    avahi-daemon \
    orphan-sysvinit-scripts \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* && \
    echo 'Port 2022' >> /etc/ssh/sshd_config && \
    echo 'PermitRootLogin yes' >> /etc/ssh/sshd_config && \
    echo 'PasswordAuthentication no' >> /etc/ssh/sshd_config && \
    sed -i 's/#enable-dbus=yes/enable-dbus=no/g' /etc/avahi/avahi-daemon.conf

RUN apt-get update && apt-get install -y --no-install-recommends \
    zsh \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN git config --global http.postBuffer 524288000 && \
    git config --global core.compression 0 && \
    if [ ! -d "$HOME/.oh-my-zsh" ]; then \
    sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" "" --unattended; \
    fi && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    sed -i 's/plugins=(git)/plugins=()/g' ~/.zshrc && \
    sed -i "s/# zstyle ':omz:update' mode disabled/zstyle ':omz:update' mode disabled/g" ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc && \
    chsh -s /bin/zsh root

RUN echo "source ~/env_setup.bash" >> ~/.bashrc

RUN mkdir -p /dogbot_install/ /root/.ssh

COPY --from=dogbot-develop --chown=root:root /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys
RUN chmod 600 /root/.ssh/authorized_keys

COPY .script/template/entrypoint /entrypoint
COPY .script/template/dogbot-service /etc/init.d/dogbot

COPY .script/template/env_setup.bash /root/env_setup.bash
COPY .script/template/env_setup.zsh /root/env_setup.zsh

WORKDIR /root/

ENV need_compile=True
ENV HOST=localhost
ENV MASTER=/
ENV DEPTH_CAMERA_TYPE=Dabai
ENV MACHINE_TYPE=JetRover_Acker

ENTRYPOINT ["tini", "--"]
CMD ["/entrypoint"]
