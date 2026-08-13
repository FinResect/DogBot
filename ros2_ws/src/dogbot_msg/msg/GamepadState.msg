# 手柄（USB 游戏手柄）整体状态消息，由 dogbot_core 的 Gamepad 设备类发布。
# 轴/扳机值均归一化；按键按下方 BUTTON_* 常量索引。

# ===== 连接状态 =====
uint8 UNKNOWN=0        # 初始状态：从未检测到设备，所有状态量无效
uint8 CONNECTED=1      # 设备已连接，所有状态量有效
uint8 DISCONNECTED=2   # 设备已断开，所有状态量无效
uint8 status           # 当前连接状态，取值见上方枚举

# ===== 摇杆 =====
Stick2D joystick_left   # 左摇杆，x/y 均归一化 [-1,1]
Stick2D joystick_right  # 右摇杆，x/y 均归一化 [-1,1]

# ===== 模拟扳机 =====
float32 trigger_left    # 左扳机模拟值 [0,1]：0=完全松开，1=按到底
float32 trigger_right   # 右扳机模拟值 [0,1]：0=完全松开，1=按到底

# ===== 十字键 =====
int8 hat_x              # 十字键水平：-1=左，0=居中，1=右
int8 hat_y              # 十字键垂直：-1=下，0=居中，1=上

# ===== 按键索引常量（Xbox 兼容布局）=====
int32 BUTTON_A=0        # A 键（下方，确认）
int32 BUTTON_B=1        # B 键（右侧，返回）
int32 BUTTON_X=2        # X 键（左侧）
int32 BUTTON_Y=3        # Y 键（上方）
int32 BUTTON_L1=4       # 左上肩键 L1
int32 BUTTON_R1=5       # 右上肩键 R1
int32 BUTTON_L2=6       # 左下肩键 L2
int32 BUTTON_R2=7       # 右下肩键 R2
int32 BUTTON_SELECT=8   # 选择键 SELECT
int32 BUTTON_START=9    # 开始键 START
int32 BUTTON_MODE=10    # 模式键 MODE
int32 BUTTON_L3=11      # 左摇杆按下
int32 BUTTON_R3=12      # 右摇杆按下

# ===== 按键状态 =====
int32[] buttons         # 按键状态数组，按上方 BUTTON_* 常量索引：1=按下，0=松开
