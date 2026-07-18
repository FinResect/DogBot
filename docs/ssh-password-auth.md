# SSH 密码认证配置指南

## 当前认证机制

DogBot 的宿主机与 runtime 容器之间通过 SSH 通信，用于 `ssh-remote` / `attach-remote` / `sync-remote`（unison）等操作。默认配置为**公钥认证**：

### 客户端（宿主机）

`~/.ssh/config` 中的 `Host remote` 段：

```
Host remote
    HostName <目标IP>
    Port 2022
    User root
    PreferredAuthentications publickey    ← 仅用公钥
    IdentityFile ~/.ssh/id_rsa            ← 指定私钥
```

### 服务端（runtime 容器）

Dockerfile 中：

```dockerfile
echo 'PasswordAuthentication no' >> /etc/ssh/sshd_config
COPY --from=dogbot-develop /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys
```

- sshd 禁用密码登录
- 仅接受与 develop 镜像中私钥配对的公钥

---

## 改为密码认证

### 需要改动的文件

| 文件 | 改动内容 |
|---|---|
| `DogBot/.ssh/config` | 移除 `PreferredAuthentications` 和 `IdentityFile` |
| `DogBot/Dockerfile` runtime 段 | `PasswordAuthentication no` → `yes` |
| `DogBot/Dockerfile` runtime 段 | 新增 `chpasswd` 设置 root 密码 |

脚本层（`ssh-remote`、`sync-remote`、`set-remote`）无需修改——它们不涉及认证方式。

---

### 步骤 1：修改 SSH 客户端配置

编辑 `DogBot/.ssh/config`，删掉以下两行（或注释掉）：

```diff
 Host remote
     HostName 127.0.0.1
     Port 2022
     User root
-    PreferredAuthentications publickey
-    IdentityFile ~/.ssh/id_rsa
     AddressFamily inet
```

> SSH 默认就尝试密码认证，不需要显式声明 `PreferredAuthentications password`。如果你希望明确指定，可添加 `PreferredAuthentications keyboard-interactive,password`。

---

### 步骤 2：修改 Dockerfile（runtime 段）

编辑 `DogBot/Dockerfile`，找到 runtime 镜像构建段（约第 206 行附近）。

#### 2.1 启用密码认证

```diff
-  echo 'PasswordAuthentication no' >> /etc/ssh/sshd_config
+  echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config
```

#### 2.2 设置 root 密码

在 sshd 配置行之后新增：

```dockerfile
RUN echo 'root:<你的密码>' | chpasswd
```

#### 2.3 可选：移除 authorized_keys（不复用公钥时）

```diff
- COPY --from=dogbot-develop --chown=root:root /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys
- RUN chmod 600 /root/.ssh/authorized_keys
```

> 如果保留这两行，则同时支持**公钥 + 密码**双认证，有时调试更方便。

---

**完整改动示例（`DogBot/Dockerfile` runtime 段）：**

```dockerfile
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
    echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config && \
    sed -i 's/#enable-dbus=yes/enable-dbus=no/g' /etc/avahi/avahi-daemon.conf

RUN echo 'root:your_password_here' | chpasswd
```

---

### 步骤 3：重新构建 runtime 镜像

```bash
docker buildx build . --platform linux/arm64 --target dogbot-runtime -t dogbot-runtime:arm64 --load
```

---

### 步骤 4：部署并验证

启动容器后测试密码登录：

```bash
ssh -p 2022 root@<树莓派IP>
```

如果连接成功，`ssh-remote` 和 `sync-remote` 即可正常使用。

---

### 注意事项

1. **密码安全**：密码明文写在 Dockerfile 中，任何人拿到镜像都可以提取密码。更安全的做法是通过 `ARG` 传入：
   ```dockerfile
   ARG ROOT_PASSWORD
   RUN echo "root:${ROOT_PASSWORD}" | chpasswd
   ```
   构建时：
   ```bash
   docker build --build-arg ROOT_PASSWORD=xxx --target dogbot-runtime ...
   ```

2. **被同步的产物**：`sync-remote` 使用 unison，每次连接都会提示密码。可以配合 `sshpass` 自动输入（不推荐），或使用 SSH ControlMaster 复用连接减少输入次数。

3. **双认证模式**：保留 `authorized_keys` 的 COPY 行，可以在宿主机开发时继续用密钥免密登录，生产中改用密码。sshd 兼容两种方式同时生效。
