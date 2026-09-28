# WebotsGimbal

`WebotsGimbal` 是 Webots 里的云台下位机模拟器。

它不负责算打哪里，也不负责规划轨迹。它只做一件事：拿到 Aimer 给出的云台目标，再根据 Webots
里的姿态和角速度反馈，按固定周期（默认 1 ms）给俯仰、yaw 两个电机输出力矩。

## 为什么不用位置控制

Webots 里 `target_motor_pitch` 和 `target_motor_yaw` 的关节零位，不等于相机启动时应该看的方向。

如果直接 `setPosition(0)`，或者把目标角直接写给 Webots 的位置伺服，相机会被拉到另一个姿态，
目标会直接跑出画面。所以这个模块只用力矩模式：

- 第一次接管控制时对两个电机调用 `setPosition(infinity)` 并把力矩置零。
- 之后只通过 `setTorque()` 输出两轴力矩。

## 输入

主输入是 `host/target_euler`。

它和 DevC `HostData::HostGimbalTarget` 同布局（9 个 `float`），包含 roll/pitch/yaw、角速度和
角加速度。模块只使用 `rol`/`yaw` 及其速度、加速度；`pit` 字段只用于匹配 DevC 数据布局。
含非有限值的目标会被忽略并打印警告。

当 `rol == 0 && yaw == 0` 时，语义与 DevC `HostData` 一致：上位机不接管云台。模块会清空当前目标、
复位 PID，并让控制线程把两轴力矩置零。

反馈输入有两个：

- `host/gimbal_quat`（`LibXR::Quaternion<float>`）：当前云台姿态，和 C 板回传 topic 保持一致，
  由 `QDU-Robomaster/WebotsCamera` 发布。
- `libxr_def_domain/camera_gyro`（3 个 `float`）：当前角速度，取第 0 分量为俯仰角速度、第 2 分量为
  yaw 角速度；尚未收到时按 0 处理。

`camera_gyro` 和 `host/target_euler` 必须在构造前存在，否则模块记录错误并抛出
`std::runtime_error`；`host/gimbal_quat` 不存在时由本模块创建。

## 控制过程

模块内部有一个 `WebotsGimbalCtl` 线程（REALTIME 优先级，栈 8192），每 `control_period_ms`
运行一次，默认 1 ms，也就是 1000 Hz。

topic 回调不做控制，只缓存最新值。控制线程每周期取一次快照，然后在锁外完成 PID、前馈、限幅和
`setTorque()`。这样 topic 抖动不会把控制计算和 Webots API 调用塞在同一把锁里。没有有效目标或
尚未收到姿态反馈时，两轴力矩置零。

单轴控制链路是：

1. 角度环：目标角和当前角相减，得到目标角速度。
2. 速度前馈：把 `host/target_euler` 中的目标角速度直接叠加到目标角速度。
3. 角速度环：目标角速度和陀螺仪反馈相减，得到反馈力矩。
4. 模型前馈：叠加惯量前馈（目标角加速度 × 惯量）和两轴库伦/粘滞摩擦补偿。
5. 力矩限幅：最后写给 Webots 电机。

## 坐标和符号

`host/gimbal_quat` 来自 WebotsCamera 发布的相机姿态。云台命令使用右手系，`x` 向右、`y` 向前、
`z` 向上；yaw 以前向为 0、左转为正。

`host/target_euler` 使用 DevC HostData 字段名；当前两轴云台只使用 `rol` 和 `yaw`，`pit` 保留为 0。
机械俯仰轴是右手系 `+X` 轴，对应 ZYX 欧拉角 roll；yaw 是 `+Z` 轴。

当前 Webots world 的 pitch HingeJoint 正方向会让相机低头，而公开命令的 `+rol` 表示抬头；因此控制器
只在写 Webots pitch 电机力矩时取反。yaw 轴与公开命令同号。这个取反只用于匹配 Webots 关节轴方向，
不改变 `host/target_euler` 的语义。

## 看日志

周期日志前缀是 `WebotsGimbal ctrl`，每 `log_interval` 次控制输出打印一条。

里面保留了目标角、目标速度、目标加速度、反馈角、角度误差、角速度反馈、角速度目标和最终力矩。
这个日志是给离线画曲线用的，字段顺序不要随手改。

## 设计约束

- 这里只消费最新 topic，不做轨迹队列。
- 不用 Webots 位置伺服替代力矩控制。
- 不把 PID 生成的目标速度再差分成加速度前馈。
- 不在 topic 回调里做 PID 或调用 Webots 电机 API。
- 当前不使用固定 pitch 重力补偿；惯量前馈和两轴摩擦补偿是模型固定项。改 world 里的质量、重心、
  摩擦、相机安装姿态或关节轴以后，先重新标定力矩方向和补偿项，再调 PID。

## 依赖

无其他模块依赖。

外部依赖：Webots。模块直接调用 Webots C++ API（`webots/Robot.hpp`、`webots/Motor.hpp`），只能在
LibXR Webots 后端下构建（CI 使用 `-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots
-DWEBOTS_HOME=/usr/local/webots`）。

## 构造接口

```cpp
WebotsGimbal(const Param& param = {.pid_pitch_angle = DefaultPitchAnglePid(),
                                   .pid_pitch_omega = DefaultPitchOmegaPid(),
                                   .pid_yaw_angle = DefaultYawAnglePid(),
                                   .pid_yaw_omega = DefaultYawOmegaPid(),
                                   .pitch_inertia = 0.00012f, .yaw_inertia = 0.0002f,
                                   .pitch_torque_limit = 0.035f,
                                   .yaw_torque_limit = 0.04f,
                                   .control_period_ms = 1, .log_interval = 1000});
```

无依赖项。

配置（`Param`）：

- `pid_pitch_angle` / `pid_yaw_angle`：角度环 `LibXR::PID<float>::Param`，输出目标角速度。默认
  `DefaultPitchAnglePid()`（`p = 16`，`out_limit = 10`）/ `DefaultYawAnglePid()`（`p = 8`，
  `out_limit = 10`，`cycle = true`）。
- `pid_pitch_omega` / `pid_yaw_omega`：角速度环，输出电机力矩。默认 `DefaultPitchOmegaPid()`
  （`p = 0.012`，`i = 0.04`，`i_limit = 0.08`，`out_limit = 0.035`）/ `DefaultYawOmegaPid()`
  （`p = 0.02`，`i = 0.08`，`i_limit = 0.08`，`out_limit = 0.04`）。
- `pitch_inertia` / `yaw_inertia`：惯量前馈系数，默认 `0.00012` / `0.0002`。
- `pitch_torque_limit` / `yaw_torque_limit`：最终电机力矩限幅，单位 N·m，默认 `0.035` / `0.04`。
- `control_period_ms`：内部控制线程周期，单位 ms，默认 `1`，最小按 1 执行。
- `log_interval`：每多少次控制输出打一条日志，默认 `1000`；设为 `0` 关闭周期日志。

## 使用

```sh
xrobot module add QDU-Robomaster/WebotsGimbal
xrobot setup
xrobot instance add QDU-Robomaster/WebotsGimbal
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出。
本模块没有依赖项，按需修改 `param`：

```yaml
modules:
  - module: QDU-Robomaster/WebotsGimbal
    id: webotsgimbal_0
    args:
      - param:
          pid_pitch_angle: WebotsGimbal::DefaultPitchAnglePid()
          pid_pitch_omega: WebotsGimbal::DefaultPitchOmegaPid()
          pid_yaw_angle: WebotsGimbal::DefaultYawAnglePid()
          pid_yaw_omega: WebotsGimbal::DefaultYawOmegaPid()
          pitch_inertia: 0.00012f
          yaw_inertia: 0.0002f
          pitch_torque_limit: 0.035f
          yaw_torque_limit: 0.04f
          control_period_ms: '1'
          log_interval: '1000'
```

本模块不使用 BSP 对象，不需要 `XR_REGISTER`。发布 `libxr_def_domain/camera_gyro` 的
`QDU-Robomaster/WebotsCamera` 实例（`device_name` 为 `"camera"`，`raw_topic_domain_name` 为
`"libxr_def_domain"`）和创建 `host/target_euler` 的实例（如 `QDU-Robomaster/Aimer`）必须在
`modules:` 中排在本实例之前。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/WebotsGimbal`
（在 BSP 中）打印当前的构造函数。
