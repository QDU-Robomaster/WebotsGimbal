# WebotsGimbal

`WebotsGimbal` 是 Webots 里的云台下位机模拟器。

它不负责算打哪里，也不负责规划轨迹。它只做一件事：拿到上游给出的云台目标，再根据 Webots 里的姿态和角速度反馈，按 1ms 周期给 pitch/yaw 两个电机输出力矩。

## 为什么不用位置控制

Webots 里 `target_motor_pitch` 和 `target_motor_yaw` 的关节零位，不等于相机启动时应该看的方向。

如果直接 `setPosition(0)`，或者把目标角直接写给 Webots 的位置伺服，相机会被拉到另一个姿态，目标会直接跑出画面。所以这个模块只用力矩模式：

- 启动时对电机调用 `setPosition(infinity)`。
- 之后只通过 `setTorque()` 输出两轴力矩。

## 输入

主输入是 `host/target_euler`。

它和 DevC `HostData::HostGimbalTarget` 同布局，包含 roll/pitch/yaw、角速度和角加速度。模块只使用 `rol`/`yaw` 及其速度、加速度；`pit` 字段保留为 ABI 对齐。

当 `rol == 0 && yaw == 0` 时，语义与 DevC `HostData` 一致：上位机不接管云台。模块会清空当前目标、复位 PID，并让控制线程把两轴力矩置零。

反馈输入有两个：

- `host/gimbal_quat`：当前云台姿态，和 C 板回传 topic 保持一致。
- `libxr_def_domain/camera_gyro`：当前角速度。

## 控制过程

模块内部有一个 `WebotsGimbalCtl` 线程，默认每 1ms 跑一次，也就是 1000Hz。

topic 回调不做控制，只缓存最新值。控制线程每周期取一次快照，然后在锁外完成 PID、前馈、限幅和 `setTorque()`。这样 topic 抖动不会把控制计算和 Webots API 调用塞在同一把锁里。

单轴控制链路是：

1. 角度环：目标角和当前角相减，得到目标角速度。
2. 速度前馈：把 `host/target_euler` 中的目标角速度直接叠加到目标角速度。
3. 角速度环：目标角速度和陀螺仪反馈相减，得到反馈力矩。
4. 模型前馈：叠加惯量和两轴摩擦补偿。
5. 力矩限幅：最后写给 Webots 电机。

`host/target_euler` 的 `rol == 0 && yaw == 0` 表示上游不要接管云台。

## 坐标和符号

`host/gimbal_quat` 来自 WebotsCamera 发布的相机姿态。控制侧命令坐标系是 `x` 右、`y` 前、`z` 上。

`host/target_euler` 沿用历史 ABI 名称；当前两轴云台只使用 `rol` 和 `yaw`，`pit` 保留为 0。机械俯仰轴是右手系 `+X` 轴，对应 ZYX 欧拉角 roll；yaw 是 `+Z` 轴，前向为 0、左转为正。

坐标约定：云台命令使用右手系，`x` 向右、`y` 向前、`z` 向上；yaw 以前向为 0、左转为正。

当前 Webots world 的 pitch HingeJoint 正方向会让相机低头，而公开命令的
`+rol` 表示抬头；因此控制器只在写 Webots pitch 电机力矩时取反。yaw 轴与公开命令同号。
这个取反是 Webots 关节轴适配，不改变 `host/target_euler` 的语义。

## 参数怎么理解

- `pid_pitch_angle` / `pid_yaw_angle`：角度环，输出目标角速度。
- `pid_pitch_omega` / `pid_yaw_omega`：角速度环，输出电机力矩。
- `pitch_inertia` / `yaw_inertia`：惯量前馈系数。
- `pitch_torque_limit` / `yaw_torque_limit`：最终电机力矩限幅。
- `control_period_ms`：内部控制线程周期，默认 `1` ms。
- `log_interval`：每多少次控制输出打一条日志；设为 `0` 可以关闭周期日志。

当前 Webots 云台不使用固定 pitch 重力补偿；惯量前馈和两轴摩擦补偿是模型固定项，没有单独做成配置。改 world 里的质量、重心、摩擦或关节轴以后，需要重新标定这些固定项。

## 看日志

周期日志前缀是 `WebotsGimbal ctrl`。

里面保留了目标角、目标速度、目标加速度、反馈角、角度误差、角速度反馈、角速度目标和最终力矩。这个日志是给离线画曲线用的，字段顺序不要随手改。

## 改这个模块时要守住的边界

- 不要把上游轨迹队列塞进这里；这里永远只消费最新 topic。
- 不要用 Webots 位置伺服替代力矩控制。
- 不要把 PID 生成的目标速度再差分成加速度前馈。
- 不要在 topic 回调里做 PID 或调用 Webots 电机 API。
- 如果相机安装姿态、关节轴或 world 物理参数变了，先做力矩方向和补偿项标定，再调 PID。
