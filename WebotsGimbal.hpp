#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots torque-mode gimbal controller
constructor_args:
  - pid_pitch_angle:
      k: 1.0
      p: 16.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 10.0
      cycle: false
  - pid_pitch_omega:
      k: 1.0
      p: 0.012
      i: 0.04
      d: 0.0
      i_limit: 0.08
      out_limit: 0.035
      cycle: false
  - pid_yaw_angle:
      k: 1.0
      p: 8.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 10.0
      cycle: true
  - pid_yaw_omega:
      k: 1.0
      p: 0.02
      i: 0.08
      d: 0.0
      i_limit: 0.08
      out_limit: 0.04
      cycle: false
  - pitch_inertia: 0.00012
  - yaw_inertia: 0.0002
  - pitch_torque_limit: 0.035
  - yaw_torque_limit: 0.04
  - control_period_ms: 1
  - log_interval: 1000
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

/**
 * @file WebotsGimbal.hpp
 * @brief Webots 云台力矩控制模块。
 *
 * 本模块模拟云台下位机：接收 host/target_euler、读取 Webots 相机姿态和角速度，
 * 在独立 1ms 控制线程中输出 pitch/yaw 两轴电机力矩。
 */

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>

#include "app_framework.hpp"
/**
 * @brief DevC HostData 接收的云台目标载荷。
 */
struct WebotsHostGimbalTarget
{
  float rol{0.0f};       ///< roll 命令，单位 rad。
  float pit{0.0f};       ///< pitch 命令，单位 rad。
  float yaw{0.0f};       ///< yaw 命令，单位 rad。
  float rol_dot{0.0f};   ///< roll 速度前馈，单位 rad/s。
  float pit_dot{0.0f};   ///< pitch 速度前馈，单位 rad/s。
  float yaw_dot{0.0f};   ///< yaw 速度前馈，单位 rad/s。
  float rol_ddot{0.0f};  ///< roll 加速度前馈，单位 rad/s^2。
  float pit_ddot{0.0f};  ///< pitch 加速度前馈，单位 rad/s^2。
  float yaw_ddot{0.0f};  ///< yaw 加速度前馈，单位 rad/s^2。
};

static_assert(sizeof(WebotsHostGimbalTarget) == sizeof(float) * 9);

#include "libxr.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "transform.hpp"

/**
 * @brief Webots Robot 全局句柄，由 Webots 系统层提供。
 */
extern webots::Robot *_libxr_webots_robot_handle;

/**
 * @brief Webots 云台力矩控制器。
 *
 * topic 回调只缓存最新命令和反馈；固定周期控制线程独占 PID 状态和 Webots 电机输出。
 */
class WebotsGimbal : public LibXR::Application
{
  /**
   * @brief `camera_gyro` topic 的原始三轴角速度样本。
   */
  using GyroSample = std::array<float, 3>;

  /**
   * @brief 当前有效目标命令。
   *
   * 该结构只保存控制线程需要的目标角、目标角速度和目标角加速度。
   */
  struct TargetCommand
  {
    bool valid{false};      ///< 当前是否有可执行目标。
    float pitch{0.0f};      ///< 目标 pitch 角，单位 rad。
    float yaw{0.0f};        ///< 目标 yaw 角，单位 rad。
    float pitch_vel{0.0f};  ///< 计划 pitch 角速度，单位 rad/s。
    float yaw_vel{0.0f};    ///< 计划 yaw 角速度，单位 rad/s。
    float pitch_acc{0.0f};  ///< 计划 pitch 角加速度，单位 rad/s^2。
    float yaw_acc{0.0f};    ///< 计划 yaw 角加速度，单位 rad/s^2。
  };

  /**
   * @brief 最新云台反馈。
   *
   * 姿态来自 `gimbal/rotation`，角速度来自 `camera_gyro`。如果角速度尚未到达，
   * 控制线程会用 0 作为速度反馈。
   */
  struct FeedbackState
  {
    bool has_rotation{false};  ///< 是否已经收到姿态反馈。
    bool has_gyro{false};      ///< 是否已经收到陀螺仪反馈。
    float pitch{0.0f};         ///< 当前 pitch 角，单位 rad。
    float yaw{0.0f};           ///< 当前 yaw 角，单位 rad。
    float pitch_rate{0.0f};    ///< 当前 pitch 角速度，单位 rad/s。
    float yaw_rate{0.0f};      ///< 当前 yaw 角速度，单位 rad/s。
  };

  /**
   * @brief 控制线程每周期使用的一份输入快照。
   *
   * 控制线程只在取快照时持有互斥锁，后续 PID 计算和 Webots 电机调用都在锁外执行。
   */
  struct ControlSnapshot
  {
    TargetCommand target{};      ///< 目标命令快照。
    FeedbackState feedback{};    ///< 反馈状态快照。
    bool reset_requested{false};  ///< 回调侧请求控制器复位。
  };

  /**
   * @brief Webots 电机索引。
   */
  enum class MotorType
  {
    PITCH,   ///< pitch 轴电机。
    YAW,     ///< yaw 轴电机。
    NUMBER   ///< 电机数量。
  };

  static constexpr const char *MOTOR_NAMES[2] = {
      "target_motor_pitch",
      "target_motor_yaw"};  ///< Webots world 中的 pitch/yaw 电机名。
  static constexpr float PITCH_TORQUE_SIGN = -1.0f;  ///< pitch 力矩方向标定。
  static constexpr float YAW_TORQUE_SIGN = 1.0f;     ///< yaw 力矩方向标定。
  static constexpr float PITCH_GRAVITY_TORQUE =
      0.012f;  ///< pitch 固定重力补偿幅值，单位 Nm。
  static constexpr float PITCH_GRAVITY_OFFSET =
      0.0f;  ///< pitch 重力补偿角度偏置，单位 rad。
  static constexpr float PITCH_COULOMB_TORQUE =
      0.0008f;  ///< pitch 库伦摩擦补偿幅值，单位 Nm。
  static constexpr float PITCH_VISCOUS_TORQUE =
      0.003f;  ///< pitch 粘滞摩擦补偿系数。
  static constexpr float YAW_COULOMB_TORQUE =
      0.0008f;  ///< yaw 库伦摩擦补偿幅值，单位 Nm。
  static constexpr float YAW_VISCOUS_TORQUE =
      0.004f;  ///< yaw 粘滞摩擦补偿系数。
  static constexpr float FRICTION_DEADBAND_RAD_S =
      0.02f;  ///< 摩擦方向判定死区，单位 rad/s。
  static constexpr float YAW_FORWARD_OFFSET_RAD =
      static_cast<float>(M_PI / 2.0);  ///< 从右轴 yaw 转为前轴 yaw 的偏移。

 public:
  /**
   * @brief 返回默认 pitch 角度环 PID 参数。
   * @return 默认 pitch 角度环参数。
   */
  static constexpr LibXR::PID<float>::Param DefaultPitchAnglePid()
  {
    return LibXR::PID<float>::Param{1.0f, 16.0f, 0.0f, 0.0f, 0.0f, 10.0f,
                                    false};
  }

  /**
   * @brief 返回默认 pitch 角速度环 PID 参数。
   * @return 默认 pitch 角速度环参数。
   */
  static constexpr LibXR::PID<float>::Param DefaultPitchOmegaPid()
  {
    return LibXR::PID<float>::Param{1.0f, 0.012f, 0.04f, 0.0f, 0.08f, 0.035f,
                                    false};
  }

  /**
   * @brief 返回默认 yaw 角度环 PID 参数。
   * @return 默认 yaw 角度环参数。
   */
  static constexpr LibXR::PID<float>::Param DefaultYawAnglePid()
  {
    return LibXR::PID<float>::Param{1.0f, 8.0f, 0.0f, 0.0f, 0.0f, 10.0f,
                                    true};
  }

  /**
   * @brief 返回默认 yaw 角速度环 PID 参数。
   * @return 默认 yaw 角速度环参数。
   */
  static constexpr LibXR::PID<float>::Param DefaultYawOmegaPid()
  {
    return LibXR::PID<float>::Param{1.0f, 0.02f, 0.08f, 0.0f, 0.08f, 0.04f,
                                    false};
  }

  /**
   * @brief 构造 Webots 云台控制器并启动内部控制线程。
   * @param hardware 硬件容器，本模块当前不直接取硬件对象。
   * @param app 应用管理器，用于注册本模块。
   * @param pid_pitch_angle pitch 角度环 PID 参数，输出目标 pitch 角速度。
   * @param pid_pitch_omega pitch 角速度环 PID 参数，输出 pitch 电机力矩。
   * @param pid_yaw_angle yaw 角度环 PID 参数，输出目标 yaw 角速度。
   * @param pid_yaw_omega yaw 角速度环 PID 参数，输出 yaw 电机力矩。
   * @param pitch_inertia pitch 轴惯量前馈系数。
   * @param yaw_inertia yaw 轴惯量前馈系数。
   * @param pitch_torque_limit pitch 最终力矩限幅，单位 Nm。
   * @param yaw_torque_limit yaw 最终力矩限幅，单位 Nm。
   * @param control_period_ms 内部控制线程周期，单位 ms，最小值为 1。
   * @param log_interval 控制日志间隔；为 0 时关闭周期日志。
   */
  WebotsGimbal(
      LibXR::HardwareContainer &hardware, LibXR::ApplicationManager &app,
      LibXR::PID<float>::Param pid_pitch_angle = DefaultPitchAnglePid(),
      LibXR::PID<float>::Param pid_pitch_omega = DefaultPitchOmegaPid(),
      LibXR::PID<float>::Param pid_yaw_angle = DefaultYawAnglePid(),
      LibXR::PID<float>::Param pid_yaw_omega = DefaultYawOmegaPid(),
      float pitch_inertia = 0.00012f, float yaw_inertia = 0.0002f,
      float pitch_torque_limit = 0.035f, float yaw_torque_limit = 0.04f,
      uint32_t control_period_ms = 1, uint32_t log_interval = 1000)
      : pid_pitch_angle_(pid_pitch_angle),
        pid_pitch_omega_(pid_pitch_omega),
        pid_yaw_angle_(pid_yaw_angle),
        pid_yaw_omega_(pid_yaw_omega),
        pitch_inertia_(pitch_inertia),
        yaw_inertia_(yaw_inertia),
        pitch_torque_limit_(pitch_torque_limit),
        yaw_torque_limit_(yaw_torque_limit),
        control_period_ms_(std::max<uint32_t>(1U, control_period_ms)),
        log_interval_(log_interval)
  {
    (void)hardware;
    RegisterTopicCallbacks();
    control_thread_.Create(this, ControlThread, "WebotsGimbalCtl", 8192,
                           LibXR::Thread::Priority::REALTIME);
    app.Register(*this);
  }

  /**
   * @brief 外部监控入口。
   *
   * 控制周期由内部线程负责，因此这里保持为空。
   */
  void OnMonitor() override {}

 private:
  /**
   * @brief 注册所有输入 topic 回调。
   */
  void RegisterTopicCallbacks()
  {
    auto gyro_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          self->HandleGyroSample(data);
        },
        this);
    gyro_topic_.RegisterCallback(gyro_cb);

    auto rotation_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          self->HandleRotationSample(data);
        },
        this);
    gimbal_rotation_topic_.RegisterCallback(rotation_cb);

    auto target_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          self->HandleHostGimbalTarget(data);
        },
        this);
    host_gimbal_target_topic_.RegisterCallback(target_cb);
  }

  /**
   * @brief 固定周期控制线程入口。
   * @param self 当前控制器对象。
   */
  static void ControlThread(WebotsGimbal *self)
  {
    LibXR::MillisecondTimestamp last_wakeup = LibXR::Thread::GetTime();
    while (true)
    {
      self->ControlStep();
      LibXR::Thread::SleepUntil(last_wakeup, self->control_period_ms_);
    }
  }

  /**
   * @brief 处理陀螺仪原始数据。
   * @param data topic 原始负载，期望为 3 个 float。
   */
  void HandleGyroSample(LibXR::RawData &data)
  {
    GyroSample gyro{};
    if (data.addr_ == nullptr || data.size_ != sizeof(gyro))
    {
      return;
    }

    std::memcpy(gyro.data(), data.addr_, sizeof(gyro));

    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback_.pitch_rate = gyro[1];
    feedback_.yaw_rate = gyro[2];
    feedback_.has_gyro = true;
  }

  /**
   * @brief 处理云台姿态反馈。
   * @param data topic 原始负载，期望为 `LibXR::Quaternion<float>`。
   */
  void HandleRotationSample(LibXR::RawData &data)
  {
    if (data.addr_ == nullptr ||
        data.size_ != sizeof(LibXR::Quaternion<float>))
    {
      return;
    }

    const auto rotation = *static_cast<LibXR::Quaternion<float> *>(data.addr_);
    const auto euler = rotation.ToEulerAngleZYX();

    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback_.pitch = euler[1];
    feedback_.yaw = LimitRad(euler[2] + YAW_FORWARD_OFFSET_RAD);
    feedback_.has_rotation = true;
  }

  /**
   * @brief 处理 host/target_euler 云台目标。
   * @param data topic 原始负载，期望为 `WebotsHostGimbalTarget`。
   */
  void HandleHostGimbalTarget(LibXR::RawData &data)
  {
    if (data.addr_ == nullptr || data.size_ != sizeof(WebotsHostGimbalTarget))
    {
      return;
    }

    WebotsHostGimbalTarget target{};
    std::memcpy(&target, data.addr_, sizeof(target));

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (std::abs(target.pit) < 1e-6f && std::abs(target.yaw) < 1e-6f)
    {
      ClearTargetCommandLocked();
      return;
    }

    if (!IsFinite(target))
    {
      XR_LOG_WARN("WebotsGimbal ignored non-finite host/target_euler");
      return;
    }

    if (!target_command_.valid)
    {
      XR_LOG_INFO(
          "WebotsGimbal first host/target_euler pitch=%f yaw=%f pitch_vel=%f yaw_vel=%f",
          static_cast<double>(target.pit), static_cast<double>(target.yaw),
          static_cast<double>(target.pit_dot),
          static_cast<double>(target.yaw_dot));
    }

    target_command_.valid = true;
    target_command_.pitch = target.pit;
    target_command_.yaw = target.yaw;
    target_command_.pitch_vel = target.pit_dot;
    target_command_.yaw_vel = target.yaw_dot;
    target_command_.pitch_acc = target.pit_ddot;
    target_command_.yaw_acc = target.yaw_ddot;
  }

  /**
   * @brief 拷贝控制线程本周期使用的输入快照。
   * @return 当前目标、反馈和复位请求。
   */
  ControlSnapshot TakeControlSnapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ControlSnapshot snapshot{};
    snapshot.target = target_command_;
    snapshot.feedback = feedback_;
    snapshot.reset_requested = control_reset_requested_;
    control_reset_requested_ = false;
    return snapshot;
  }

  /**
   * @brief 执行一次 1ms 控制周期。
   */
  void ControlStep()
  {
    const auto snapshot = TakeControlSnapshot();
    if (snapshot.reset_requested)
    {
      StopTorqueOutput();
      ResetControlTimeline();
    }

    if (!snapshot.target.valid || !snapshot.feedback.has_rotation)
    {
      StopTorqueOutput();
      return;
    }

    if (!motors_enabled_)
    {
      EnableMotors();
    }

    const auto now = LibXR::Timebase::GetMicroseconds();
    if (!have_last_control_time_)
    {
      last_control_time_ = now;
      have_last_control_time_ = true;
      ResetControlState();
      return;
    }

    const float dt = (now - last_control_time_).ToSecondf();
    last_control_time_ = now;
    if (!std::isfinite(dt) || dt <= 0.0f)
    {
      return;
    }

    UpdateTorque(snapshot, dt);
  }

  /**
   * @brief 第一次接管控制时把 Webots 电机切到力矩模式。
   */
  void EnableMotors()
  {
    if (motors_enabled_)
    {
      return;
    }

    for (size_t i = 0; i < static_cast<size_t>(MotorType::NUMBER); i++)
    {
      motors_[i] = _libxr_webots_robot_handle->getMotor(MOTOR_NAMES[i]);
      // 当前 world 的关节零位和相机初始视角不一致，所以不能用绝对位置伺服。
      motors_[i]->setPosition(std::numeric_limits<double>::infinity());
      motors_[i]->setTorque(0.0);
    }
    motors_enabled_ = true;
    torque_output_active_ = false;
    ResetControlState();
  }

  /**
   * @brief 根据输入快照计算并输出两轴电机力矩。
   * @param snapshot 控制输入快照。
   * @param dt 本次控制周期时长，单位 s。
   */
  void UpdateTorque(const ControlSnapshot &snapshot, float dt)
  {
    const auto &target = snapshot.target;
    const auto &feedback = snapshot.feedback;
    const float pitch_rate = feedback.has_gyro ? feedback.pitch_rate : 0.0f;
    const float yaw_rate = feedback.has_gyro ? feedback.yaw_rate : 0.0f;
    const float target_pitch_vel = target.pitch_vel;
    const float target_yaw_vel = target.yaw_vel;
    const float target_pitch_acc = target.pitch_acc;
    const float target_yaw_acc = target.yaw_acc;
    const float pitch_error = LimitRad(target.pitch - feedback.pitch);
    const float yaw_error = LimitRad(target.yaw - feedback.yaw);

    const float target_pitch_omega =
        pid_pitch_angle_.Calculate(pitch_error, 0.0f, dt) + target_pitch_vel;
    const float target_yaw_omega =
        pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt) + target_yaw_vel;

    // PID 微分项是反馈项；不要对角度环生成的目标速度再差分后当成前馈加速度。
    const float pitch_feed_forward =
        pitch_inertia_ * target_pitch_acc +
        PitchGravityCompensation(feedback.pitch) +
        FrictionCompensation(target_pitch_omega, pitch_rate, PITCH_COULOMB_TORQUE,
                             PITCH_VISCOUS_TORQUE);
    const float yaw_feed_forward =
        yaw_inertia_ * target_yaw_acc +
        FrictionCompensation(target_yaw_omega, yaw_rate, YAW_COULOMB_TORQUE,
                             YAW_VISCOUS_TORQUE);
    pid_pitch_omega_.SetFeedForward(pitch_feed_forward);
    pid_yaw_omega_.SetFeedForward(yaw_feed_forward);

    // 当前 world 标定结果：pitch 电机正力矩会让发布坐标系的 pitch 变小。
    const float pitch_torque =
        Clamp(PITCH_TORQUE_SIGN *
                  pid_pitch_omega_.Calculate(target_pitch_omega, pitch_rate, dt),
              -pitch_torque_limit_, pitch_torque_limit_);
    const float yaw_torque =
        Clamp(YAW_TORQUE_SIGN *
                  pid_yaw_omega_.Calculate(target_yaw_omega, yaw_rate, dt),
              -yaw_torque_limit_, yaw_torque_limit_);

    motors_[static_cast<size_t>(MotorType::PITCH)]->setTorque(pitch_torque);
    motors_[static_cast<size_t>(MotorType::YAW)]->setTorque(yaw_torque);
    torque_output_active_ = true;

    command_count_++;
    if (log_interval_ != 0U && (command_count_ % log_interval_) == 0U)
    {
      XR_LOG_INFO(
          "WebotsGimbal ctrl c=%u dt=%.4f py=%.4f pyv=%.4f pya=%.2f yy=%.4f yyv=%.4f yya=%.2f fp=%.4f fy=%.4f ep=%.4f ey=%.4f rp=%.3f ry=%.3f op=%.3f oy=%.3f tp=%.4f ty=%.4f",
          command_count_, static_cast<double>(dt),
          static_cast<double>(target.pitch),
          static_cast<double>(target_pitch_vel),
          static_cast<double>(target_pitch_acc),
          static_cast<double>(target.yaw), static_cast<double>(target_yaw_vel),
          static_cast<double>(target_yaw_acc),
          static_cast<double>(feedback.pitch), static_cast<double>(feedback.yaw),
          static_cast<double>(pitch_error), static_cast<double>(yaw_error),
          static_cast<double>(pitch_rate), static_cast<double>(yaw_rate),
          static_cast<double>(target_pitch_omega),
          static_cast<double>(target_yaw_omega),
          static_cast<double>(pitch_torque),
          static_cast<double>(yaw_torque));
    }
  }

  /**
   * @brief 复位 PID 内部状态和前馈项。
   */
  void ResetControlState()
  {
    pid_pitch_angle_.Reset();
    pid_pitch_omega_.Reset();
    pid_yaw_angle_.Reset();
    pid_yaw_omega_.Reset();
    pid_pitch_omega_.SetFeedForward(0.0f);
    pid_yaw_omega_.SetFeedForward(0.0f);
  }

  /**
   * @brief 清空当前目标命令并请求控制线程复位。
   *
   * 调用方必须已经持有 `state_mutex_`。
   */
  void ClearTargetCommandLocked()
  {
    target_command_ = {};
    control_reset_requested_ = true;
  }

  /**
   * @brief 清空控制时间轴并复位控制器状态。
   */
  void ResetControlTimeline()
  {
    have_last_control_time_ = false;
    ResetControlState();
  }

  /**
   * @brief 停止输出电机力矩。
   */
  void StopTorqueOutput()
  {
    const bool should_reset = have_last_control_time_ || torque_output_active_;
    if (motors_enabled_ && torque_output_active_)
    {
      motors_[static_cast<size_t>(MotorType::PITCH)]->setTorque(0.0);
      motors_[static_cast<size_t>(MotorType::YAW)]->setTorque(0.0);
      torque_output_active_ = false;
    }

    if (should_reset)
    {
      ResetControlTimeline();
    }
  }

  /**
   * @brief 将数值限制在闭区间内。
   * @param value 输入值。
   * @param min_value 最小值。
   * @param max_value 最大值。
   * @return 限幅后的值。
   */
  static float Clamp(float value, float min_value, float max_value)
  {
    if (value < min_value)
    {
      return min_value;
    }
    if (value > max_value)
    {
      return max_value;
    }
    return value;
  }

  /**
   * @brief 将角度归一化到 [-pi, pi]。
   * @param angle 输入角度，单位 rad。
   * @return 归一化后的角度，单位 rad。
   */
  static float LimitRad(float angle)
  {
    while (angle > static_cast<float>(M_PI))
    {
      angle -= static_cast<float>(2.0 * M_PI);
    }
    while (angle < static_cast<float>(-M_PI))
    {
      angle += static_cast<float>(2.0 * M_PI);
    }
    return angle;
  }

  /**
   * @brief 计算 pitch 轴固定重力补偿力矩。
   * @param pitch 当前 pitch 角，单位 rad。
   * @return 重力补偿力矩，单位 Nm。
   */
  static float PitchGravityCompensation(float pitch)
  {
    return -PITCH_GRAVITY_TORQUE * std::cos(pitch + PITCH_GRAVITY_OFFSET);
  }

  /**
   * @brief 计算库伦摩擦和粘滞摩擦补偿。
   * @param target_omega 目标角速度，单位 rad/s。
   * @param measured_omega 实测角速度，单位 rad/s。
   * @param coulomb_torque 库伦摩擦补偿幅值，单位 Nm。
   * @param viscous_torque 粘滞摩擦补偿系数。
   * @return 摩擦补偿力矩，单位 Nm。
   */
  static float FrictionCompensation(float target_omega, float measured_omega,
                                    float coulomb_torque, float viscous_torque)
  {
    float direction = 0.0f;
    if (std::abs(target_omega) > FRICTION_DEADBAND_RAD_S)
    {
      direction = target_omega > 0.0f ? 1.0f : -1.0f;
    }
    else if (std::abs(measured_omega) > FRICTION_DEADBAND_RAD_S)
    {
      direction = measured_omega > 0.0f ? 1.0f : -1.0f;
    }

    return coulomb_torque * direction + viscous_torque * target_omega;
  }

  /**
   * @brief 检查 host 云台目标中的控制量是否全为有限值。
   * @param target 待检查的云台目标。
   * @return 全部控制量有限时返回 true。
   */
  static bool IsFinite(const WebotsHostGimbalTarget &target)
  {
    return std::isfinite(target.rol) && std::isfinite(target.pit) &&
           std::isfinite(target.yaw) && std::isfinite(target.rol_dot) &&
           std::isfinite(target.pit_dot) && std::isfinite(target.yaw_dot) &&
           std::isfinite(target.rol_ddot) && std::isfinite(target.pit_ddot) &&
           std::isfinite(target.yaw_ddot);
  }

  std::mutex state_mutex_{};  ///< 保护 topic 回调写入的最新输入状态。
  TargetCommand target_command_{};  ///< 最新目标命令。
  FeedbackState feedback_{};        ///< 最新云台反馈。
  bool control_reset_requested_{false};  ///< 是否请求控制线程复位。

  webots::Motor *motors_[static_cast<size_t>(
      MotorType::NUMBER)]{};       ///< pitch/yaw 两轴 Webots 电机指针。
  bool motors_enabled_{false};     ///< 电机是否已经切入力矩模式。
  bool torque_output_active_{false};  ///< 当前是否正在输出非空控制力矩。
  uint32_t command_count_{0};        ///< 已输出控制命令计数。
  LibXR::MicrosecondTimestamp last_control_time_{0};  ///< 上次控制时间。
  bool have_last_control_time_{false};  ///< 是否已有有效上次控制时间。
  LibXR::PID<float> pid_pitch_angle_;   ///< pitch 角度环 PID。
  LibXR::PID<float> pid_pitch_omega_;   ///< pitch 角速度环 PID。
  LibXR::PID<float> pid_yaw_angle_;     ///< yaw 角度环 PID。
  LibXR::PID<float> pid_yaw_omega_;     ///< yaw 角速度环 PID。
  float pitch_inertia_{0.00012f};       ///< pitch 轴惯量前馈系数。
  float yaw_inertia_{0.0002f};          ///< yaw 轴惯量前馈系数。
  float pitch_torque_limit_{0.035f};    ///< pitch 力矩限幅，单位 Nm。
  float yaw_torque_limit_{0.04f};       ///< yaw 力矩限幅，单位 Nm。
  uint32_t control_period_ms_{1};       ///< 控制线程周期，单位 ms。
  uint32_t log_interval_{1000};         ///< 周期日志间隔。
  LibXR::Thread control_thread_{};      ///< 固定周期控制线程。

  /** @brief 原始数据 topic 域。 */
  LibXR::Topic::Domain raw_topic_domain_ =
      LibXR::Topic::Domain("libxr_def_domain");
  /** @brief 相机陀螺仪 topic。 */
  LibXR::Topic gyro_topic_ =
      LibXR::Topic::FindOrCreate<GyroSample>("camera_gyro", &raw_topic_domain_);
  /** @brief DevC host topic 域。 */
  LibXR::Topic::Domain host_domain_ = LibXR::Topic::Domain("host");
  /** @brief DevC HostData 云台目标 topic。 */
  LibXR::Topic host_gimbal_target_topic_ =
      LibXR::Topic::FindOrCreate<WebotsHostGimbalTarget>("target_euler",
                                                         &host_domain_);
  /** @brief 云台反馈 topic 域。 */
  LibXR::Topic::Domain gimbal_domain_ = LibXR::Topic::Domain("gimbal");
  /** @brief 云台姿态反馈 topic。 */
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation",
                                                           &gimbal_domain_);
};
