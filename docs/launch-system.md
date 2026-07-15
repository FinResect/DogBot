# 启动系统使用说明

## 架构

```
entrypoint → dogbot-service → ros2 launch dogbot_start start.launch.py
                                    │
                                    ├── 读取 dogbot_core/config/nodes.yaml  → 启动所有节点
                                    ├── 读取 serial_bridge/config/nodes.yaml → 启动所有节点
                                    └── ...
```

- **`dogbot_start`**：顶层编排器，只管一个包名列表，不管每个包内部有多少节点
- **各 C++ 包**：在自己的 `config/nodes.yaml` 里声明自己有哪些可执行文件

---

## 添加一个新包

编辑 `dogbot_start/launch/start.launch.py`：

```python
PACKAGES = (
    'dogbot_core',
    'serial_bridge',
    'your_new_package',   # ← 加一行
)
```

然后在 `your_new_package/config/nodes.yaml` 里声明自己的节点（见下方格式）。

---

## nodes.yaml 格式

### 最简写法（只写可执行文件名）

```yaml
nodes:
  - executable: dogbot_core_node
  - executable: imu_driver
```

每个 `executable` 对应 `CMakeLists.txt` 里 `add_executable(xxx ...)` 的名字。

### 完整参数

```yaml
nodes:
  - executable: dogbot_core_node    # [必填] CMake add_executable 里的名字
    # name: dogbot_core             # [可省略] ROS2 节点名，默认等于 executable
    # output: screen                 # [可省略] 日志输出位置，全局默认为 'screen'
```

| 参数 | 含义 | 默认值 | 必填 |
|---|---|---|---|
| `executable` | `add_executable(xxx ...)` 的名字，告诉 launch 跑哪个二进制 | 无 | **是** |
| `name` | ROS2 节点名（`ros2 node list` 看到的） | 等于 `executable` | 否 |
| `output` | 日志写到 `screen`（屏幕/stdout）还是 `log`（`~/.ros/log/`） | 全局默认：`screen` | 否 |

3 个里面有 1 个必填，2 个可省略。

---

## 修改全局默认值

编辑 `dogbot_start/launch/start.launch.py` 顶部的 `DEFAULTS` 字典：

```python
DEFAULTS = {
    'output': 'screen',    # 全局默认：所有节点默认输出到屏幕
}
```

修改一次，所有包的节点都生效。单个节点如果写了 `output: log`，以 YAML 里的为准。

---

## 示例

### 当一个包有多个节点

```yaml
# dogbot_core/config/nodes.yaml
nodes:
  - executable: dogbot_core_node
  - executable: imu_driver
  - executable: motor_controller
```

启动顺序 = YAML 里从上到下的顺序。

### 当你只想改某个节点的输出

```yaml
nodes:
  - executable: dogbot_core_node
  - executable: imu_driver
    output: log             # 这个节点不刷屏
```

其他没写 `output` 的节点继续用全局默认 `screen`。

### 当你想自定义节点名

```yaml
nodes:
  - executable: dogbot_core_node
    name: core              # ros2 node list 里看到的是 /core
```

---

## 启动方式

在容器内：

```bash
ros2 launch dogbot_start start.launch.py
```

不需要参数，它会自动读取 `PACKAGES` 列表里每个包的 `config/nodes.yaml`。
