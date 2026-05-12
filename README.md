# WebotsGimbal

`WebotsGimbal` 在 Webots 中模拟云台下位机。

当前实现不把 `target_eulr` 直接写入 Webots 电机位置。原因是现有 world
里相机的正确初始视角并不等于 `target_motor_pitch/yaw` 的关节零位；如果调用
`setPosition(0)` 或按绝对角位置控制，Webots 会把相机拉到另一个姿态，导致视觉目标直接出画面。

## Topic

- 输入 `tracker/gimbal_plan`：优先使用的云台计划，包含目标角、计划角速度和角加速度。
- 输入 `tracker/target_eulr`：兼容角度目标。
- 输入 `gimbal/rotation`：WebotsCamera 发布的相机/云台姿态。
- 输入 `libxr_def_domain/camera_gyro`：WebotsCamera 每个 world step 发布的原始角速度。

## 控制语义

- 第一个兼容 `target_eulr` 零目标会被忽略，避免注册期默认值被当成真实回零命令。
- 收到第一条有效 `gimbal_plan` 或 `target_eulr` 后才接管电机；一旦收到
  `gimbal_plan` topic，兼容 `target_eulr` 不再接管，避免失控帧后又追旧角度。
- `gimbal_plan.control=false` 会清空当前目标并复位控制器；电机力矩由 1ms 控制线程置零。
- 电机进入 torque 模式：`setPosition(infinity)`，然后通过 `setTorque()` 输出控制量。
- 力矩控制由模块内部 `WebotsGimbalCtl` 线程以 `control_period_ms=1`
  独立执行，即 1000 Hz 控制节拍；BSP 全局 `MonitorAll()` 不参与控制周期。
  话题回调只缓存最新目标、姿态和陀螺仪反馈，不排队、不外推；首次启用电机和后续
  `setTorque()` 都由控制线程执行。
- 云台角度反馈来自 `gimbal/rotation`。WebotsCamera 发布坐标系是
  `x` 右、`y` 前、`z` 上；ZYX Euler yaw 描述的是 `x` 轴朝向，
  因此控制器把 yaw 转成前向 `y` 轴语义后再与瞄准 yaw 比较，pitch
  仍使用同一 IMU 姿态下的 ZYX pitch。
- 控制器是两环串级：
  - 角度环根据目标角和当前角生成目标角速度。
  - `gimbal_plan` 的计划角速度直接叠加到目标角速度。
  - 角速度环根据目标角速度和 `camera_gyro` 反馈生成电机力矩；默认带小积分项，
    用来消除 Webots 摩擦、重力偏置和固定前馈模型误差造成的静差。
  - 前馈项包含三部分：`gimbal_plan` 的计划角加速度形成的惯量前馈、pitch 轴固定重力偏置、两轴固定摩擦补偿。
- PID 反馈链路不会再对目标角速度做有限差分；PID 的微分项属于反馈，不作为前馈。
- 标定结果显示：`target_motor_pitch` 的正力矩会让发布坐标系 pitch 变小，所以 pitch 轴最终力矩符号与 yaw 不同。

## 参数

- `pid_pitch_angle` / `pid_yaw_angle`：角度环参数，输出目标角速度。
- `pid_pitch_omega` / `pid_yaw_omega`：角速度环参数，输出电机力矩。
- `pitch_inertia` / `yaw_inertia`：与当前 Webots 云台物理模型匹配的惯量前馈系数。
- `pitch_torque_limit` / `yaw_torque_limit`：最终电机力矩限幅。
- `control_period_ms`：内部控制线程周期，默认 `1` ms。
- `log_interval`：力矩日志间隔；默认 1000 次控制输出记录一次，设为 `0`
  可关闭周期日志。

固定模型项不暴露为配置：pitch 重力偏置和两轴摩擦补偿跟当前
`auto_aim_test_field_target_vehicle_camera_preview.wbt` 中的 self 云台质量、
重心和关节摩擦一起维护。

## 后续边界

如果后续调整 world 里的 pitch 关节轴或相机姿态，需要重新做一次力矩脉冲标定，确认 motor 力矩方向与发布坐标系角速度的关系。
