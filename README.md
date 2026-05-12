# WebotsGimbal

`WebotsGimbal` 是 Webots 里的云台下位机模拟器。

它不负责算打哪里，也不负责规划轨迹。它只做一件事：拿到上游给出的云台目标，再根据 Webots 里的姿态和角速度反馈，按 1ms 周期给 pitch/yaw 两个电机输出力矩。

## 为什么不用位置控制

Webots 里 `target_motor_pitch` 和 `target_motor_yaw` 的关节零位，不等于相机启动时应该看的方向。

如果直接 `setPosition(0)`，或者把目标角直接写给 Webots 的位置伺服，相机会被拉到另一个姿态，目标会直接跑出画面。所以这个模块只用力矩模式：

- 启动时对电机调用 `setPosition(infinity)`。
- 之后只通过 `setTorque()` 输出两轴力矩。

## 输入

主输入是 `tracker/gimbal_plan`。

它包含目标角、目标角速度和目标角加速度。模块会用目标角做闭环，用目标速度和目标加速度做前馈。

兼容输入是 `tracker/target_eulr`。

它只提供目标角，没有速度和加速度。这个入口只用于旧链路或最小测试。一旦收到过 `gimbal_plan`，模块就不再听 `target_eulr`，避免上游失控时又追旧角度。

反馈输入有两个：

- `gimbal/rotation`：当前云台姿态。
- `libxr_def_domain/camera_gyro`：当前角速度。

## 控制过程

模块内部有一个 `WebotsGimbalCtl` 线程，默认每 1ms 跑一次，也就是 1000Hz。

topic 回调不做控制，只缓存最新值。控制线程每周期取一次快照，然后在锁外完成 PID、前馈、限幅和 `setTorque()`。这样 topic 抖动不会把控制计算和 Webots API 调用塞在同一把锁里。

单轴控制链路是：

1. 角度环：目标角和当前角相减，得到目标角速度。
2. 计划速度前馈：如果输入来自 `gimbal_plan`，把计划角速度直接叠加到目标角速度。
3. 角速度环：目标角速度和陀螺仪反馈相减，得到反馈力矩。
4. 模型前馈：叠加惯量、pitch 重力补偿和两轴摩擦补偿。
5. 力矩限幅：最后写给 Webots 电机。

`gimbal_plan.control=false` 表示上游不要接管云台。模块会清空当前目标、复位 PID，并让控制线程把两轴力矩置零。

## 坐标和符号

`gimbal/rotation` 来自 WebotsCamera 发布的相机姿态。当前发布坐标系是 `x` 右、`y` 前、`z` 上。

ZYX Euler 的 yaw 描述的是 `x` 轴朝向，而瞄准 yaw 使用的是前向 `y` 轴语义，所以模块在比较 yaw 前加了固定 `pi/2` 偏移。pitch 仍使用同一姿态里的 ZYX pitch。

当前 world 标定结果是：pitch 电机正力矩会让发布坐标系里的 pitch 变小，所以 pitch 输出力矩符号和 yaw 不同。

## 参数怎么理解

- `pid_pitch_angle` / `pid_yaw_angle`：角度环，输出目标角速度。
- `pid_pitch_omega` / `pid_yaw_omega`：角速度环，输出电机力矩。
- `pitch_inertia` / `yaw_inertia`：惯量前馈系数。
- `pitch_torque_limit` / `yaw_torque_limit`：最终电机力矩限幅。
- `control_period_ms`：内部控制线程周期，默认 `1` ms。
- `log_interval`：每多少次控制输出打一条日志；设为 `0` 可以关闭周期日志。

pitch 重力补偿和两轴摩擦补偿是当前 Webots 云台模型的一部分，没有单独做成配置。改 world 里的质量、重心、摩擦或关节轴以后，需要重新标定这些固定项。

## 看日志

周期日志前缀是 `WebotsGimbal ctrl`。

里面保留了目标角、目标速度、目标加速度、反馈角、角度误差、角速度反馈、角速度目标和最终力矩。这个日志是给离线画曲线用的，字段顺序不要随手改。

## 改这个模块时要守住的边界

- 不要把上游轨迹队列塞进这里；这里永远只消费最新 topic。
- 不要用 Webots 位置伺服替代力矩控制。
- 不要把 PID 生成的目标速度再差分成加速度前馈。
- 不要在 topic 回调里做 PID 或调用 Webots 电机 API。
- 如果相机安装姿态、关节轴或 world 物理参数变了，先做力矩方向和补偿项标定，再调 PID。
