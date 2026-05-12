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
#if __has_include("GimbalPlan.hpp")
#include "GimbalPlan.hpp"
#else
#ifndef QDU_GIMBAL_PLAN_HPP
#define QDU_GIMBAL_PLAN_HPP
struct GimbalPlan
{
  uint64_t image_timestamp_us{0};
  bool control{false};
  bool fire{false};
  float target_yaw{0.0f};
  float target_pitch{0.0f};
  float yaw{0.0f};
  float yaw_vel{0.0f};
  float yaw_acc{0.0f};
  float pitch{0.0f};
  float pitch_vel{0.0f};
  float pitch_acc{0.0f};
};
#endif  // QDU_GIMBAL_PLAN_HPP
#endif
#include "libxr.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "transform.hpp"

extern webots::Robot *_libxr_webots_robot_handle;

/**
 * @brief Torque-mode Webots gimbal controller.
 *
 * Topic callbacks cache the latest command and feedback samples. A dedicated
 * fixed-rate thread owns PID state and Webots motor torque output.
 */
class WebotsGimbal : public LibXR::Application
{
  using GyroSample = std::array<float, 3>;

  struct TargetCommand
  {
    bool valid{false};
    bool from_plan{false};
    float pitch{0.0f};
    float yaw{0.0f};
    float pitch_vel{0.0f};
    float yaw_vel{0.0f};
    float pitch_acc{0.0f};
    float yaw_acc{0.0f};
  };

  struct FeedbackState
  {
    bool has_rotation{false};
    bool has_gyro{false};
    float pitch{0.0f};
    float yaw{0.0f};
    float pitch_rate{0.0f};
    float yaw_rate{0.0f};
  };

  struct ControlSnapshot
  {
    TargetCommand target{};
    FeedbackState feedback{};
    bool reset_requested{false};
  };

  enum class MotorType
  {
    PITCH,
    YAW,
    NUMBER
  };

  static constexpr const char *MOTOR_NAMES[2] = {"target_motor_pitch",
                                                 "target_motor_yaw"};
  static constexpr float PITCH_TORQUE_SIGN = -1.0f;
  static constexpr float YAW_TORQUE_SIGN = 1.0f;
  static constexpr float PITCH_GRAVITY_TORQUE = 0.012f;
  static constexpr float PITCH_GRAVITY_OFFSET = 0.0f;
  static constexpr float PITCH_COULOMB_TORQUE = 0.0008f;
  static constexpr float PITCH_VISCOUS_TORQUE = 0.003f;
  static constexpr float YAW_COULOMB_TORQUE = 0.0008f;
  static constexpr float YAW_VISCOUS_TORQUE = 0.004f;
  static constexpr float FRICTION_DEADBAND_RAD_S = 0.02f;
  static constexpr float YAW_FORWARD_OFFSET_RAD =
      static_cast<float>(M_PI / 2.0);

 public:
  static constexpr LibXR::PID<float>::Param DefaultPitchAnglePid()
  {
    return LibXR::PID<float>::Param{1.0f, 16.0f, 0.0f, 0.0f, 0.0f, 10.0f,
                                    false};
  }

  static constexpr LibXR::PID<float>::Param DefaultPitchOmegaPid()
  {
    return LibXR::PID<float>::Param{1.0f, 0.012f, 0.04f, 0.0f, 0.08f, 0.035f,
                                    false};
  }

  static constexpr LibXR::PID<float>::Param DefaultYawAnglePid()
  {
    return LibXR::PID<float>::Param{1.0f, 8.0f, 0.0f, 0.0f, 0.0f, 10.0f,
                                    true};
  }

  static constexpr LibXR::PID<float>::Param DefaultYawOmegaPid()
  {
    return LibXR::PID<float>::Param{1.0f, 0.02f, 0.08f, 0.0f, 0.08f, 0.04f,
                                    false};
  }

  WebotsGimbal(
      LibXR::HardwareContainer &, LibXR::ApplicationManager &app,
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
    RegisterTopicCallbacks();
    control_thread_.Create(this, ControlThread, "WebotsGimbalCtl", 8192,
                           LibXR::Thread::Priority::REALTIME);
    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
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

    auto plan_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          self->HandleGimbalPlan(data);
        },
        this);
    gimbal_plan_topic_.RegisterCallback(plan_cb);

    auto target_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          self->HandleTargetEuler(data);
        },
        this);
    target_eulr_topic_.RegisterCallback(target_cb);
  }

  static void ControlThread(WebotsGimbal *self)
  {
    LibXR::MillisecondTimestamp last_wakeup = LibXR::Thread::GetTime();
    while (true)
    {
      self->ControlStep();
      LibXR::Thread::SleepUntil(last_wakeup, self->control_period_ms_);
    }
  }

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

  void HandleGimbalPlan(LibXR::RawData &data)
  {
    if (data.addr_ == nullptr || data.size_ != sizeof(GimbalPlan))
    {
      return;
    }

    const auto plan = *static_cast<GimbalPlan *>(data.addr_);

    std::lock_guard<std::mutex> lock(state_mutex_);
    plan_topic_seen_ = true;
    if (!plan.control)
    {
      ClearTargetCommandLocked();
      return;
    }

    if (!IsFinite(plan))
    {
      XR_LOG_WARN("WebotsGimbal ignored non-finite gimbal_plan");
      return;
    }

    if (!target_command_.from_plan)
    {
      XR_LOG_INFO(
          "WebotsGimbal first gimbal_plan pitch=%f yaw=%f pitch_vel=%f yaw_vel=%f",
          static_cast<double>(plan.pitch), static_cast<double>(plan.yaw),
          static_cast<double>(plan.pitch_vel),
          static_cast<double>(plan.yaw_vel));
    }

    target_command_.valid = true;
    target_command_.from_plan = true;
    target_command_.pitch = plan.pitch;
    target_command_.yaw = plan.yaw;
    target_command_.pitch_vel = plan.pitch_vel;
    target_command_.yaw_vel = plan.yaw_vel;
    target_command_.pitch_acc = plan.pitch_acc;
    target_command_.yaw_acc = plan.yaw_acc;
  }

  void HandleTargetEuler(LibXR::RawData &data)
  {
    if (data.addr_ == nullptr ||
        data.size_ != sizeof(LibXR::EulerAngle<float>))
    {
      return;
    }

    const auto target = *static_cast<LibXR::EulerAngle<float> *>(data.addr_);
    if (!std::isfinite(target.Pitch()) || !std::isfinite(target.Yaw()))
    {
      XR_LOG_WARN("WebotsGimbal ignored non-finite target_eulr pitch=%f yaw=%f",
                  static_cast<double>(target.Pitch()),
                  static_cast<double>(target.Yaw()));
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!target_command_.valid && std::abs(target.Pitch()) < 1e-6f &&
        std::abs(target.Yaw()) < 1e-6f)
    {
      return;
    }

    if (plan_topic_seen_)
    {
      return;
    }

    if (!target_command_.valid)
    {
      XR_LOG_INFO("WebotsGimbal first target_eulr pitch=%f yaw=%f",
                  static_cast<double>(target.Pitch()),
                  static_cast<double>(target.Yaw()));
    }

    target_command_.valid = true;
    target_command_.from_plan = false;
    target_command_.pitch = target.Pitch();
    target_command_.yaw = target.Yaw();
    target_command_.pitch_vel = 0.0f;
    target_command_.yaw_vel = 0.0f;
    target_command_.pitch_acc = 0.0f;
    target_command_.yaw_acc = 0.0f;
  }

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

  void EnableMotors()
  {
    if (motors_enabled_)
    {
      return;
    }

    for (size_t i = 0; i < static_cast<size_t>(MotorType::NUMBER); i++)
    {
      motors_[i] = _libxr_webots_robot_handle->getMotor(MOTOR_NAMES[i]);
      // The current world's joint zero does not match the camera's initial
      // view, so the module must not use absolute position servo mode.
      motors_[i]->setPosition(std::numeric_limits<double>::infinity());
      motors_[i]->setTorque(0.0);
    }
    motors_enabled_ = true;
    torque_output_active_ = false;
    ResetControlState();
  }

  void UpdateTorque(const ControlSnapshot &snapshot, float dt)
  {
    const auto &target = snapshot.target;
    const auto &feedback = snapshot.feedback;
    const float pitch_rate = feedback.has_gyro ? feedback.pitch_rate : 0.0f;
    const float yaw_rate = feedback.has_gyro ? feedback.yaw_rate : 0.0f;
    const float target_pitch_vel = target.from_plan ? target.pitch_vel : 0.0f;
    const float target_yaw_vel = target.from_plan ? target.yaw_vel : 0.0f;
    const float target_pitch_acc = target.from_plan ? target.pitch_acc : 0.0f;
    const float target_yaw_acc = target.from_plan ? target.yaw_acc : 0.0f;
    const float pitch_error = LimitRad(target.pitch - feedback.pitch);
    const float yaw_error = LimitRad(target.yaw - feedback.yaw);

    const float target_pitch_omega =
        pid_pitch_angle_.Calculate(pitch_error, 0.0f, dt) + target_pitch_vel;
    const float target_yaw_omega =
        pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt) + target_yaw_vel;

    // PID derivative terms are feedback. Do not differentiate the generated
    // target omega and reuse it as acceleration feed-forward.
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

    // Current world calibration: positive pitch motor torque decreases the
    // published-frame pitch angle.
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
          "WebotsGimbal ctrl c=%u p=%d dt=%.4f py=%.4f pyv=%.4f pya=%.2f yy=%.4f yyv=%.4f yya=%.2f fp=%.4f fy=%.4f ep=%.4f ey=%.4f rp=%.3f ry=%.3f op=%.3f oy=%.3f tp=%.4f ty=%.4f",
          command_count_, target.from_plan ? 1 : 0, static_cast<double>(dt),
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

  void ResetControlState()
  {
    pid_pitch_angle_.Reset();
    pid_pitch_omega_.Reset();
    pid_yaw_angle_.Reset();
    pid_yaw_omega_.Reset();
    pid_pitch_omega_.SetFeedForward(0.0f);
    pid_yaw_omega_.SetFeedForward(0.0f);
  }

  void ClearTargetCommandLocked()
  {
    target_command_ = {};
    control_reset_requested_ = true;
  }

  void ResetControlTimeline()
  {
    have_last_control_time_ = false;
    ResetControlState();
  }

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

  static float PitchGravityCompensation(float pitch)
  {
    return -PITCH_GRAVITY_TORQUE * std::cos(pitch + PITCH_GRAVITY_OFFSET);
  }

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

  static bool IsFinite(const GimbalPlan &plan)
  {
    return std::isfinite(plan.pitch) && std::isfinite(plan.yaw) &&
           std::isfinite(plan.pitch_vel) && std::isfinite(plan.yaw_vel) &&
           std::isfinite(plan.pitch_acc) && std::isfinite(plan.yaw_acc);
  }

  // Topic callbacks write these fields; the control thread copies them once
  // per cycle and runs without holding this mutex.
  std::mutex state_mutex_{};
  TargetCommand target_command_{};
  FeedbackState feedback_{};
  bool plan_topic_seen_{false};
  bool control_reset_requested_{false};

  // Owned by the fixed-rate control thread.
  webots::Motor *motors_[static_cast<size_t>(MotorType::NUMBER)]{};
  bool motors_enabled_{false};
  bool torque_output_active_{false};
  uint32_t command_count_{0};
  LibXR::MicrosecondTimestamp last_control_time_{0};
  bool have_last_control_time_{false};
  LibXR::PID<float> pid_pitch_angle_;
  LibXR::PID<float> pid_pitch_omega_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  float pitch_inertia_{0.00012f};
  float yaw_inertia_{0.0002f};
  float pitch_torque_limit_{0.035f};
  float yaw_torque_limit_{0.04f};
  uint32_t control_period_ms_{1};
  uint32_t log_interval_{1000};
  LibXR::Thread control_thread_{};

  LibXR::Topic::Domain raw_topic_domain_ =
      LibXR::Topic::Domain("libxr_def_domain");
  LibXR::Topic gyro_topic_ =
      LibXR::Topic::FindOrCreate<GyroSample>("camera_gyro", &raw_topic_domain_);
  LibXR::Topic::Domain tracker_domain_ = LibXR::Topic::Domain("tracker");
  LibXR::Topic target_eulr_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::EulerAngle<float>>("target_eulr",
                                                           &tracker_domain_);
  LibXR::Topic gimbal_plan_topic_ =
      LibXR::Topic::FindOrCreate<GimbalPlan>("gimbal_plan", &tracker_domain_);
  LibXR::Topic::Domain gimbal_domain_ = LibXR::Topic::Domain("gimbal");
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation",
                                                           &gimbal_domain_);
};
