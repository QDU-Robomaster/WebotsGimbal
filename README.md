# WebotsGimbal

`WebotsGimbal` 在 Webots 中模拟云台下位机。

当前实现不把 `target_eulr` 直接写入 Webots 电机位置。原因是现有 world
里相机的正确初始视角并不等于 `target_motor_pitch/yaw` 的关节零位；如果调用
`setPosition(0)` 或按绝对角位置控制，Webots 会把相机拉到另一个姿态，导致视觉目标直接出画面。

## Topic

- 输入 `gimbal_plan`：优先使用的 SP-style 云台计划，包含目标角、计划角速度和角加速度。
- 输入 `target_eulr`：SharedTopic 转发后的云台目标角。
- 输入 `gimbal/rotation`：WebotsCamera 发布的相机/云台姿态。
- 输入 `camera_gyro`：WebotsCamera 每个 world step 发布的原始角速度。

## 控制语义

- 第一个默认零命令会被忽略，避免注册期默认值被当成真实回零命令。
- 收到第一条有效 `gimbal_plan` 或 `target_eulr` 后才接管电机；有 `gimbal_plan` 时忽略兼容 `target_eulr`。
- 电机进入 torque 模式：`setPosition(infinity)`，然后通过 `setTorque()` 输出控制量。
- yaw 轴控制：`torque = kp * yaw_error - kd * yaw_rate + kv_ff * target_yaw_vel + ka_ff * target_yaw_acc`。
- pitch 轴控制：`torque = -kp * pitch_error + kd * pitch_rate - kv_ff * target_pitch_vel - ka_ff * target_pitch_acc`。
  - 标定结果显示：`target_motor_pitch` 的正力矩会让发布坐标系 pitch 变小，所以 pitch 轴的 P/D 符号与 yaw 不同。

## 后续边界

如果后续调整 world 里的 pitch 关节轴或相机姿态，需要重新做一次力矩脉冲标定，确认 motor 力矩方向与发布坐标系角速度的关系。
