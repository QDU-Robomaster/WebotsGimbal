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

class WebotsGimbal : public LibXR::Application
{
  using GyroSample = std::array<float, 3>;

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
    auto gyro_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          GyroSample gyro{};
          if (data.addr_ == nullptr || data.size_ != sizeof(gyro))
          {
            return;
          }
          std::memcpy(gyro.data(), data.addr_, sizeof(gyro));
          std::lock_guard<std::mutex> lock(self->state_mutex_);
          self->pitch_rate_ = gyro[1];
          self->yaw_rate_ = gyro[2];
          self->have_gyro_ = true;
        },
        this);
    gyro_topic_.RegisterCallback(gyro_cb);

    auto rotation_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          if (data.addr_ == nullptr ||
              data.size_ != sizeof(LibXR::Quaternion<float>))
          {
            return;
          }

          const auto rotation = *static_cast<LibXR::Quaternion<float> *>(data.addr_);
          const auto euler = rotation.ToEulerAngleZYX();
          std::lock_guard<std::mutex> lock(self->state_mutex_);
          self->current_pitch_ = euler[1];
          self->current_yaw_ = LimitRad(euler[2] + YAW_FORWARD_OFFSET_RAD);
          self->have_rotation_ = true;
        },
        this);
    gimbal_rotation_topic_.RegisterCallback(rotation_cb);

    auto plan_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          if (data.addr_ == nullptr || data.size_ != sizeof(GimbalPlan))
          {
            return;
          }

          const auto plan = *static_cast<GimbalPlan *>(data.addr_);
          std::lock_guard<std::mutex> lock(self->state_mutex_);
          self->plan_topic_seen_ = true;
          if (!plan.control)
          {
            self->ClearTargetCommand();
            return;
          }

          if (!std::isfinite(plan.pitch) || !std::isfinite(plan.yaw) ||
              !std::isfinite(plan.pitch_vel) || !std::isfinite(plan.yaw_vel) ||
              !std::isfinite(plan.pitch_acc) || !std::isfinite(plan.yaw_acc))
          {
            XR_LOG_WARN("WebotsGimbal ignored non-finite gimbal_plan");
            return;
          }

          if (!self->have_plan_)
          {
            XR_LOG_INFO(
                "WebotsGimbal first gimbal_plan pitch=%f yaw=%f pitch_vel=%f yaw_vel=%f",
                static_cast<double>(plan.pitch), static_cast<double>(plan.yaw),
                static_cast<double>(plan.pitch_vel),
                static_cast<double>(plan.yaw_vel));
          }

          self->target_pitch_ = plan.pitch;
          self->target_yaw_ = plan.yaw;
          self->target_pitch_vel_ = plan.pitch_vel;
          self->target_yaw_vel_ = plan.yaw_vel;
          self->target_pitch_acc_ = plan.pitch_acc;
          self->target_yaw_acc_ = plan.yaw_acc;
          self->have_plan_ = true;
          self->have_target_ = true;
        },
        this);
    gimbal_plan_topic_.RegisterCallback(plan_cb);

    auto target_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          if (data.addr_ == nullptr ||
              data.size_ != sizeof(LibXR::EulerAngle<float>))
          {
            return;
          }

          const auto target = *static_cast<LibXR::EulerAngle<float> *>(data.addr_);
          std::lock_guard<std::mutex> lock(self->state_mutex_);
          if (!std::isfinite(target.Pitch()) || !std::isfinite(target.Yaw()))
          {
            XR_LOG_WARN("WebotsGimbal ignored non-finite target_eulr pitch=%f yaw=%f",
                        static_cast<double>(target.Pitch()),
                        static_cast<double>(target.Yaw()));
            return;
          }

          if (!self->have_target_ && std::abs(target.Pitch()) < 1e-6f &&
              std::abs(target.Yaw()) < 1e-6f)
          {
            return;
          }

          if (self->plan_topic_seen_)
          {
            return;
          }

          if (!self->have_target_)
          {
            XR_LOG_INFO("WebotsGimbal first target_eulr pitch=%f yaw=%f",
                        static_cast<double>(target.Pitch()),
                        static_cast<double>(target.Yaw()));
          }

          self->target_pitch_ = target.Pitch();
          self->target_yaw_ = target.Yaw();
          self->target_pitch_vel_ = 0.0f;
          self->target_yaw_vel_ = 0.0f;
          self->target_pitch_acc_ = 0.0f;
          self->target_yaw_acc_ = 0.0f;
          self->have_target_ = true;
        },
        this);
    target_eulr_topic_.RegisterCallback(target_cb);

    control_thread_.Create(this, ControlThread, "WebotsGimbalCtl", 8192,
                           LibXR::Thread::Priority::REALTIME);
    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
  static void ControlThread(WebotsGimbal *self)
  {
    LibXR::MillisecondTimestamp last_wakeup = LibXR::Thread::GetTime();
    while (true)
    {
      {
        std::lock_guard<std::mutex> lock(self->state_mutex_);
        self->UpdateTorque();
      }
      LibXR::Thread::SleepUntil(last_wakeup, self->control_period_ms_);
    }
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
      // 不使用 setPosition(0) 或目标角位置控制：当前 world 的电机零位
      // 与相机正确初始视角不一致，位置伺服会把画面直接拉飞。
      motors_[i]->setPosition(std::numeric_limits<double>::infinity());
      motors_[i]->setTorque(0.0);
    }
    motors_enabled_ = true;
    torque_output_active_ = false;
    ResetControlState();
  }

  void UpdateTorque()
  {
    if (!have_target_ || !have_rotation_)
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

    const float pitch_rate = have_gyro_ ? pitch_rate_ : 0.0f;
    const float yaw_rate = have_gyro_ ? yaw_rate_ : 0.0f;
    const float target_pitch_vel = have_plan_ ? target_pitch_vel_ : 0.0f;
    const float target_yaw_vel = have_plan_ ? target_yaw_vel_ : 0.0f;
    const float target_pitch_acc = have_plan_ ? target_pitch_acc_ : 0.0f;
    const float target_yaw_acc = have_plan_ ? target_yaw_acc_ : 0.0f;
    const float pitch_error = LimitRad(target_pitch_ - current_pitch_);
    const float yaw_error = LimitRad(target_yaw_ - current_yaw_);

    const float target_pitch_omega =
        pid_pitch_angle_.Calculate(pitch_error, 0.0f, dt) + target_pitch_vel;
    const float target_yaw_omega =
        pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt) + target_yaw_vel;

    // PID derivative terms are feedback terms. Do not differentiate the PID
    // generated target omega and reuse it as acceleration feed-forward.
    const float pitch_feed_forward =
        pitch_inertia_ * target_pitch_acc +
        PitchGravityCompensation(current_pitch_) +
        FrictionCompensation(target_pitch_omega, pitch_rate, PITCH_COULOMB_TORQUE,
                             PITCH_VISCOUS_TORQUE);
    const float yaw_feed_forward =
        yaw_inertia_ * target_yaw_acc +
        FrictionCompensation(target_yaw_omega, yaw_rate, YAW_COULOMB_TORQUE,
                             YAW_VISCOUS_TORQUE);
    pid_pitch_omega_.SetFeedForward(pitch_feed_forward);
    pid_yaw_omega_.SetFeedForward(yaw_feed_forward);

    // 当前 world 标定结果：pitch motor 正力矩会让发布系 pitch 变小。
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
          command_count_, have_plan_ ? 1 : 0, static_cast<double>(dt),
          static_cast<double>(target_pitch_),
          static_cast<double>(target_pitch_vel),
          static_cast<double>(target_pitch_acc),
          static_cast<double>(target_yaw_), static_cast<double>(target_yaw_vel),
          static_cast<double>(target_yaw_acc),
          static_cast<double>(current_pitch_), static_cast<double>(current_yaw_),
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

  void ClearTargetCommand()
  {
    have_plan_ = false;
    have_target_ = false;
    target_pitch_ = 0.0f;
    target_yaw_ = 0.0f;
    target_pitch_vel_ = 0.0f;
    target_yaw_vel_ = 0.0f;
    target_pitch_acc_ = 0.0f;
    target_yaw_acc_ = 0.0f;
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
      have_last_control_time_ = false;
      ResetControlState();
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

  webots::Motor *motors_[static_cast<size_t>(MotorType::NUMBER)]{};
  std::mutex state_mutex_{};
  bool motors_enabled_{false};
  bool torque_output_active_{false};
  bool have_target_{false};
  bool have_plan_{false};
  bool plan_topic_seen_{false};
  bool have_rotation_{false};
  bool have_gyro_{false};
  uint32_t command_count_{0};
  float target_pitch_{0.0f};
  float target_yaw_{0.0f};
  float current_pitch_{0.0f};
  float current_yaw_{0.0f};
  float pitch_rate_{0.0f};
  float yaw_rate_{0.0f};
  float target_pitch_vel_{0.0f};
  float target_yaw_vel_{0.0f};
  float target_pitch_acc_{0.0f};
  float target_yaw_acc_{0.0f};
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

  LibXR::Topic::Domain raw_topic_domain_ = LibXR::Topic::Domain("libxr_def_domain");
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
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain_);
};
