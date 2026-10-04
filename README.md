# WebotsGimbal

Webots 云台力矩控制模块：按目标角与反馈为 pitch、yaw 两个电机输出力矩 / Webots gimbal torque controller Module that outputs the torques of the pitch and yaw motors from the target angles and the feedback

## 1. 模块作用 / Purpose

WebotsGimbal 是 Webots 中的云台下位机模拟器。它接收 Aimer 给出的云台目标，根据 Webots 中的姿态与角速度反馈，按固定周期（默认 1 ms）给 pitch、yaw 两个电机输出力矩。

两个电机以力矩模式工作：第一次接管控制时，对 `target_motor_pitch` 和 `target_motor_yaw` 调用 `setPosition(infinity)` 并把力矩置零，之后通过 `setTorque()` 输出两轴力矩。Webots world 中这两个关节的零位与相机启动时的朝向不同，因此由力矩而不是位置驱动。

WebotsGimbal is the gimbal controller simulator in Webots. It receives the gimbal target given by Aimer and outputs the torques of the pitch and yaw motors at a fixed period (default 1 ms) from the attitude and angular velocity feedback in Webots.

The two motors work in torque mode: when control is first taken over, `setPosition(infinity)` is called on `target_motor_pitch` and `target_motor_yaw` and the torque is set to zero, after which the two axis torques are output through `setTorque()`. The joint zero positions of these two joints in the Webots world differ from the direction the camera looks at on startup, so the motors are driven by torque instead of position.

## 2. 输入与控制 / Input and Control

主输入是 `host/target_euler`，与 DevC `HostData::HostGimbalTarget` 布局相同（9 个 `float`），包含 roll / pitch / yaw、角速度和角加速度。模块使用 `rol` / `yaw` 及其速度、加速度，`pit` 字段与 DevC 数据布局对齐。含非有限值的目标被忽略并打印警告。当 `rol == 0 && yaw == 0` 时，语义与 DevC `HostData` 一致，即上位机放弃接管云台：模块清空当前目标、复位 PID，控制线程把两轴力矩置零。

反馈输入：

- `host/gimbal_quat`（`LibXR::Quaternion<float>`）：当前云台姿态，与 C 板回传的 Topic 一致，由 `QDU-Robomaster/WebotsCamera` 发布。
- `libxr_def_domain/camera_gyro`（3 个 `float`）：当前角速度，第 0 分量为 pitch 角速度，第 2 分量为 yaw 角速度；尚未收到时按 0 处理。

`camera_gyro` 与 `host/target_euler` 在构造前已存在，缺失时记录错误并抛出 `std::runtime_error`；`host/gimbal_quat` 缺失时由本模块创建。

控制线程 `WebotsGimbalCtl`（REALTIME 优先级，栈 8192）每 `control_period_ms` 运行一次，默认 1 ms，即 1000 Hz。Topic 回调在锁内缓存最新值；控制线程每周期在锁内取一次快照，PID、前馈、限幅与 `setTorque()` 在锁外完成，持锁时间只覆盖缓存更新与取快照。没有有效目标或尚未收到姿态反馈时，两轴力矩置零。

单轴控制链路：

1. 角度环：目标角与当前角相减，得到目标角速度。
2. 速度前馈：`host/target_euler` 中的目标角速度叠加到目标角速度。
3. 角速度环：目标角速度与陀螺仪反馈相减，得到反馈力矩。
4. 模型前馈：叠加惯量前馈（目标角加速度 × 惯量）和两轴的库伦与粘滞摩擦补偿。
5. 力矩限幅后写给 Webots 电机。

模块只消费最新的 Topic 值，PID 与 Webots 电机调用都在控制线程内执行。惯量前馈和两轴摩擦补偿是模型固定项，对应当前 world 的质量、重心、摩擦、相机安装姿态与关节轴。

The main input is `host/target_euler`, with the same layout as DevC `HostData::HostGimbalTarget` (9 `float`), carrying roll / pitch / yaw, angular velocity and angular acceleration. The Module uses `rol` / `yaw` with their velocity and acceleration, and the `pit` field aligns with the DevC data layout. Targets containing non-finite values are ignored with a warning. When `rol == 0 && yaw == 0` the semantics match DevC `HostData`, meaning the host gives up control of the gimbal: the Module clears the current target, resets the PIDs and the control thread sets both axis torques to zero.

Feedback inputs:

- `host/gimbal_quat` (`LibXR::Quaternion<float>`): the current gimbal attitude, identical to the Topic returned by the C board, published by `QDU-Robomaster/WebotsCamera`.
- `libxr_def_domain/camera_gyro` (3 `float`): the current angular velocity, component 0 is the pitch rate and component 2 the yaw rate; 0 is used until the first sample arrives.

`camera_gyro` and `host/target_euler` exist before construction, and a missing one is logged and throws `std::runtime_error`; `host/gimbal_quat` is created by this Module when missing.

The control thread `WebotsGimbalCtl` (REALTIME priority, stack 8192) runs every `control_period_ms`, 1 ms by default, i.e. 1000 Hz. The Topic callbacks cache the latest values under the lock; every period the control thread takes one snapshot under the lock and performs the PID, the feedforward, the limiting and `setTorque()` outside it, so the lock is held only while the cache is updated or the snapshot is taken. Without a valid target or before the attitude feedback arrives, both axis torques are set to zero.

The single-axis control chain:

1. Angle loop: the target angle minus the current angle gives the target angular velocity.
2. Velocity feedforward: the target angular velocity from `host/target_euler` is added to the target angular velocity.
3. Angular velocity loop: the target angular velocity minus the gyroscope feedback gives the feedback torque.
4. Model feedforward: the inertia feedforward (target angular acceleration × inertia) and the Coulomb and viscous friction compensation of the two axes are added.
5. The torque is limited and written to the Webots motor.

The Module consumes only the latest Topic values, and the PIDs and the Webots motor calls run in the control thread. The inertia feedforward and the friction compensation are fixed model terms matching the mass, center of gravity, friction, camera mounting attitude and joint axes of the current world.

## 3. 坐标与符号 / Coordinates and Signs

`host/gimbal_quat` 来自 WebotsCamera 发布的相机姿态。云台命令使用右手系，`x` 向右、`y` 向前、`z` 向上；yaw 以前向为 0、左转为正。

`host/target_euler` 使用 DevC HostData 字段名；两轴云台使用 `rol` 和 `yaw`，`pit` 为 0。机械俯仰轴是右手系 `+X` 轴，对应 ZYX 欧拉角 roll；yaw 是 `+Z` 轴。

Webots world 中 pitch HingeJoint 的正方向使相机低头，而公开命令的 `+rol` 表示抬头，因此控制器在写 pitch 电机力矩时取反；yaw 轴与公开命令同号。取反用于匹配 Webots 关节轴方向，`host/target_euler` 的语义保持不变。

`host/gimbal_quat` is the camera attitude published by WebotsCamera. The gimbal command uses a right-handed frame with `x` to the right, `y` forward and `z` up; yaw is 0 at the front and positive to the left.

`host/target_euler` uses the DevC HostData field names; the two-axis gimbal uses `rol` and `yaw`, and `pit` is 0. The mechanical pitch axis is the right-handed `+X` axis, corresponding to the ZYX Euler angle roll; yaw is the `+Z` axis.

The positive direction of the pitch HingeJoint in the Webots world pitches the camera down, while `+rol` of the public command means pitching up, so the controller negates the pitch motor torque when writing it; the yaw axis has the same sign as the public command. The negation matches the Webots joint axis direction, and the semantics of `host/target_euler` stay unchanged.

## 4. 日志 / Log

周期日志前缀为 `WebotsGimbal ctrl`，每 `log_interval` 次控制输出一条。日志包含目标角、目标速度、目标加速度、反馈角、角度误差、角速度反馈、角速度目标和最终力矩，字段顺序固定，用于离线绘制曲线。

The periodic log has the prefix `WebotsGimbal ctrl` and is printed once every `log_interval` control outputs. It contains the target angle, target velocity, target acceleration, feedback angle, angle error, angular velocity feedback, target angular velocity and the final torque, in a fixed field order for plotting curves offline.

## 5. 构造接口 / Constructor

```cpp
WebotsGimbal(const Param& param = {.pid_pitch_angle = DefaultPitchAnglePid(),
                                   .pid_pitch_omega = DefaultPitchOmegaPid(),
                                   .pid_yaw_angle = DefaultYawAnglePid(),
                                   .pid_yaw_omega = DefaultYawOmegaPid(),
                                   .pitch_inertia = 0.00012f,
                                   .yaw_inertia = 0.0002f,
                                   .pitch_torque_limit = 0.035f,
                                   .yaw_torque_limit = 0.04f,
                                   .control_period_ms = 1,
                                   .log_interval = 1000});
```

依赖：无。

配置参数（`Param`；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `pid_pitch_angle` / `pid_yaw_angle`：角度环，输出目标角速度。默认 `DefaultPitchAnglePid()`（`k = 1`，`p = 16`，`out_limit = 10`，`cycle = false`）与 `DefaultYawAnglePid()`（`k = 1`，`p = 8`，`out_limit = 10`，`cycle = true`），其余字段为 0。
- `pid_pitch_omega` / `pid_yaw_omega`：角速度环，输出电机力矩。默认 `DefaultPitchOmegaPid()`（`k = 1`，`p = 0.012`，`i = 0.04`，`i_limit = 0.08`，`out_limit = 0.035`，`cycle = false`）与 `DefaultYawOmegaPid()`（`k = 1`，`p = 0.02`，`i = 0.08`，`i_limit = 0.08`，`out_limit = 0.04`，`cycle = false`），其余字段为 0。
- `pitch_inertia` / `yaw_inertia`：惯量前馈系数，默认 `0.00012` / `0.0002`。
- `pitch_torque_limit` / `yaw_torque_limit`：最终电机力矩限幅，单位 N·m，默认 `0.035` / `0.04`。
- `control_period_ms`：内部控制线程周期，单位 ms，默认 `1`，最小按 1 执行。
- `log_interval`：每多少次控制输出一条日志，默认 `1000`；为 `0` 时关闭周期日志。

Dependencies: none.

Configuration parameters (`Param`; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `pid_pitch_angle` / `pid_yaw_angle`: angle loop, output is the target angular velocity. Defaults `DefaultPitchAnglePid()` (`k = 1`, `p = 16`, `out_limit = 10`, `cycle = false`) and `DefaultYawAnglePid()` (`k = 1`, `p = 8`, `out_limit = 10`, `cycle = true`), the remaining fields are 0.
- `pid_pitch_omega` / `pid_yaw_omega`: angular velocity loop, output is the motor torque. Defaults `DefaultPitchOmegaPid()` (`k = 1`, `p = 0.012`, `i = 0.04`, `i_limit = 0.08`, `out_limit = 0.035`, `cycle = false`) and `DefaultYawOmegaPid()` (`k = 1`, `p = 0.02`, `i = 0.08`, `i_limit = 0.08`, `out_limit = 0.04`, `cycle = false`), the remaining fields are 0.
- `pitch_inertia` / `yaw_inertia`: inertia feedforward coefficients, default `0.00012` / `0.0002`.
- `pitch_torque_limit` / `yaw_torque_limit`: final motor torque limits in N·m, default `0.035` / `0.04`.
- `control_period_ms`: period of the internal control thread in ms, default `1`, at least 1 is used.
- `log_interval`: number of control outputs between log lines, default `1000`; `0` disables the periodic log.

## 6. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `host/target_euler` | 订阅 | 9 个 `float`（DevC `HostData::HostGimbalTarget`） | 云台目标角、角速度、角加速度 |
| `host/gimbal_quat` | 订阅 | `LibXR::Quaternion<float>` | 云台姿态，缺失时由本模块创建 |
| `libxr_def_domain/camera_gyro` | 订阅 | 3 个 `float` | 云台角速度 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `host/target_euler` | Subscribe | 9 `float` (DevC `HostData::HostGimbalTarget`) | Gimbal target angle, angular velocity and angular acceleration |
| `host/gimbal_quat` | Subscribe | `LibXR::Quaternion<float>` | Gimbal attitude, created by this Module when missing |
| `libxr_def_domain/camera_gyro` | Subscribe | 3 `float` | Gimbal angular velocity |

## 7. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/WebotsGimbal` 写入的实例，无依赖项，`param` 按需修改：

An instance written by `xrobot instance add QDU-Robomaster/WebotsGimbal`, which has no dependencies; `param` is adjusted as needed:

```yaml
modules:
  - module: QDU-Robomaster/WebotsGimbal
    id: WebotsGimbal_0
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
          control_period_ms: 1
          log_interval: 1000
```

发布 `libxr_def_domain/camera_gyro` 的 `QDU-Robomaster/WebotsCamera` 实例（`device_name` 为 `"camera"`，`raw_topic_domain_name` 为 `"libxr_def_domain"`）和创建 `host/target_euler` 的实例（如 `QDU-Robomaster/Aimer`）列在本实例之前。

The `QDU-Robomaster/WebotsCamera` instance that publishes `libxr_def_domain/camera_gyro` (`device_name` `"camera"`, `raw_topic_domain_name` `"libxr_def_domain"`) and the instance that creates `host/target_euler` (such as `QDU-Robomaster/Aimer`) are listed before this instance.

## 8. 依赖与硬件 / Dependencies and Hardware

依赖：

- Webots：模块调用 Webots C++ API（`webots/Robot.hpp`、`webots/Motor.hpp`），在 LibXR Webots 后端下构建（CI 使用 `-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots -DWEBOTS_HOME=/usr/local/webots`）。
- LibXR。

硬件：Webots world 中名为 `target_motor_pitch` 与 `target_motor_yaw` 的两个电机。

Dependencies:

- Webots: the Module calls the Webots C++ API (`webots/Robot.hpp`, `webots/Motor.hpp`) and is built with the LibXR Webots backend (CI uses `-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots -DWEBOTS_HOME=/usr/local/webots`).
- LibXR.

Hardware: the two motors named `target_motor_pitch` and `target_motor_yaw` in the Webots world.
