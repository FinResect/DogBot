# 计划：把 Place 重构为 turn_step / straight_step 手动编排原语（仅改 action.hpp）

## 背景

`Place` 是固定状态机（TurnRight→MoveForward→TurnBack→TurnLeft），经
`leg_controller.cpp:336` 的 `start_place` 触发，`update` 在 500Hz 下调用。
本轮重构**只改头文件**：类名保持 `Place`，外部接口（`start` / `start(Params)` /
`abort` / `update`）不变，`leg_controller.cpp` 零改动，行为等价。

## 方向约定（依据现有参数确认）

- **vx < 0 = 前进**：`Forward` → 输出 `−|vx|`；`Backward` → 输出 `+|vx|`
- **omega < 0 = 右转**：`Right` → 目标偏移 = `+turn_sign·angle`（默认
  `turn_sign=-1` → 偏移 −45°，yaw 减小 → err<0 → omega<0 → 右转）；
  `Left` → 偏移 = `−turn_sign·angle`（+45°）
- `turn_control`：`err = wrap(target − yaw)`，`omega = clamp(err·turn_gain, ±omega上限)`

## 改动：`ros2_ws/src/dogbot_core/src/controller/action/action.hpp`

`wrap_angle` / `Climb` / `Action` 管理器不动。

1. 新增命名空间级 `enum class Direction { Left, Right, Forward, Backward }`
2. `Place::Params` 修订：
   - 删除 `distance_m`（-14），新增 `straight_time_s = 5.833`（由
     14/(|vx|=3·0.8) 折算，0.8 打滑系数并入标定时长，代码中不再出现该常量）
   - 其余默认值不变：`turn_angle_deg=45`、`vx=-3`、`omega_max=1000`、
     `turn_gain=3`、`angle_tol_deg=2`、`min_turn_time=0.5`、`turn_sign=-1`
3. 新增手动编排原语（无默认参数值）：
   - `turn_step(Direction, angle_deg, omega)`：闭环转到指定角度（相对锁存起点 yaw）
   - `straight_step(Direction, vx, duration)`：定时直行
   - 参数每次调用直接生效、可任意更改（**不参与状态比较**）；仅当步骤类型或
     方向切换时才锁存起点 yaw / 重置计时，同一步骤内重复调用幂等；完成后
     `step → None`，下次调用开新步（可无缝衔接连续转圈/直行）
   - 当前生效值内部保存供 `update()` 使用（`update` 签名由 cpp 固定，参数
     无法经它传入，这是必然的最小存储）
4. 固定序列改用原语实现（行为等价）：
   - `TurnRight` → `turn_step(Right, turn_angle_deg, omega_max)`
   - `MoveForward` → 守卫（`vx≥0` 或 `straight_time_s≤0` 则跳过直行）；
     `straight_step(Forward, |vx|, straight_time_s)`
   - `TurnBack` → `begin_turn_abs(initial_yaw_, omega_max)`（私有，绝对目标，
     修正直行期间的 yaw 漂移）
   - `TurnLeft` → `turn_step(Left, turn_angle_deg, omega_max)`
5. 状态：`Step{None,Turn,Straight}`（原语）与
   `Phase{TurnRight,MoveForward,TurnBack,TurnLeft,Done}`（序列）两套私有枚举，
   共用同一 `turn_control` / `straight_control` 状态机与 `step_start_` 计时。

## 验证

无单元测试，按 AGENTS.md 运行 `./.script/build-dogbot --packages-select dogbot_core`
确认编译通过。
