# 手柄（USB 游戏手柄）整体状态消息，由 dogbot_core 的 Gamepad 设备类发布。
# 各状态量按语义分组：按键 / 摇杆 / 肩键 / 方向，全部具名字段，无索引数组。

# ===== 连接状态 =====
uint8 UNKNOWN=0        # 初始状态：从未检测到设备，所有状态量无效
uint8 CONNECTED=1      # 设备已连接，所有状态量有效
uint8 DISCONNECTED=2   # 设备已断开，所有状态量无效
uint8 status           # 当前连接状态，取值见上方枚举

# ===== 状态量分组 =====
Buttons buttons        # 按键组（A/B/X/Y、L3/R3、SELECT/START/MODE）
Sticks sticks          # 摇杆组（joystick_left/right，归一化 [-1,1]）
Shoulders shoulders    # 肩键组（L1/R1/L2/R2 数字态 + trigger_left/right 模拟值）
DPad dpad              # 方向键组（up/down/left/right，斜向=同时置位）
