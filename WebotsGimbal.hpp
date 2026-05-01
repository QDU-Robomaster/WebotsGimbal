#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

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
#include "transform.hpp"

extern webots::Robot *_libxr_webots_robot_handle;

class WebotsGimbal : public LibXR::Application
{
  struct GyroStamped
  {
    LibXR::MicrosecondTimestamp sensor_timestamp_us{};
    std::array<float, 3> angular_velocity_xyz{};
  };

  enum class MotorType
  {
    PITCH,
    YAW,
    NUMBER
  };

  static constexpr const char *MOTOR_NAMES[2] = {"target_motor_pitch",
                                                 "target_motor_yaw"};
  static constexpr float PITCH_KP = 0.008f;
  static constexpr float PITCH_KD = 0.004f;
  static constexpr float PITCH_KV_FF = 0.001f;
  static constexpr float PITCH_KA_FF = 0.00002f;
  static constexpr float PITCH_MAX_TORQUE = 0.005f;
  static constexpr float YAW_KP = 0.04f;
  static constexpr float YAW_KD = 0.015f;
  static constexpr float YAW_KV_FF = 0.005f;
  static constexpr float YAW_KA_FF = 0.00008f;
  static constexpr float YAW_MAX_TORQUE = 0.02f;

 public:
  WebotsGimbal(LibXR::HardwareContainer &, LibXR::ApplicationManager &app)
  {
    auto gyro_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          const auto gyro = *static_cast<GyroStamped *>(data.addr_);
          self->pitch_rate_ = gyro.angular_velocity_xyz[1];
          self->yaw_rate_ = gyro.angular_velocity_xyz[2];
          self->have_gyro_ = true;
          self->UpdateTorque();
        },
        this);
    gyro_topic_.RegisterCallback(gyro_cb);

    auto rotation_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          const auto rotation = *static_cast<LibXR::Quaternion<float> *>(data.addr_);
          const auto euler = rotation.ToEulerAngleZYX();
          self->current_pitch_ = euler[1];
          self->current_yaw_ = euler[2];
          self->have_rotation_ = true;
          self->UpdateTorque();
        },
        this);
    gimbal_rotation_topic_.RegisterCallback(rotation_cb);

    auto plan_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          const auto plan = *static_cast<GimbalPlan *>(data.addr_);
          if (!plan.control)
          {
            self->have_plan_ = false;
            self->target_pitch_vel_ = 0.0f;
            self->target_yaw_vel_ = 0.0f;
            self->target_pitch_acc_ = 0.0f;
            self->target_yaw_acc_ = 0.0f;
            return;
          }

          if (!std::isfinite(plan.pitch) || !std::isfinite(plan.yaw) ||
              !std::isfinite(plan.pitch_vel) || !std::isfinite(plan.yaw_vel))
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
          self->EnableMotors();
          self->UpdateTorque();
        },
        this);
    gimbal_plan_topic_.RegisterCallback(plan_cb);

    auto target_cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          const auto target = *static_cast<LibXR::EulerAngle<float> *>(data.addr_);
          if (!std::isfinite(target.Yaw()))
          {
            XR_LOG_WARN("WebotsGimbal ignored non-finite target_eulr yaw=%f",
                        static_cast<double>(target.Yaw()));
            return;
          }

          if (!self->have_target_ && std::abs(target.Pitch()) < 1e-6f &&
              std::abs(target.Yaw()) < 1e-6f)
          {
            return;
          }

          if (self->have_plan_)
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
          self->EnableMotors();
          self->UpdateTorque();
        },
        this);
    target_eulr_topic_.RegisterCallback(target_cb);

    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
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
  }

  void UpdateTorque()
  {
    if (!motors_enabled_ || !have_target_ || !have_rotation_)
    {
      return;
    }

    const float pitch_error = LimitRad(target_pitch_ - current_pitch_);
    const float yaw_error = LimitRad(target_yaw_ - current_yaw_);
    const float pitch_rate = have_gyro_ ? pitch_rate_ : 0.0f;
    const float yaw_rate = have_gyro_ ? yaw_rate_ : 0.0f;
    const float target_pitch_vel = have_plan_ ? target_pitch_vel_ : 0.0f;
    const float target_yaw_vel = have_plan_ ? target_yaw_vel_ : 0.0f;
    // 当前 world 标定结果：pitch motor 正力矩会让发布系 pitch 变小，
    // 因此 P/D 项符号与 yaw 轴相反。
    const float pitch_torque =
        Clamp(-PITCH_KP * pitch_error + PITCH_KD * pitch_rate -
                  PITCH_KV_FF * target_pitch_vel - PITCH_KA_FF * target_pitch_acc_,
              -PITCH_MAX_TORQUE, PITCH_MAX_TORQUE);
    const float yaw_torque =
        Clamp(YAW_KP * yaw_error - YAW_KD * yaw_rate +
                  YAW_KV_FF * target_yaw_vel + YAW_KA_FF * target_yaw_acc_,
              -YAW_MAX_TORQUE, YAW_MAX_TORQUE);

    motors_[static_cast<size_t>(MotorType::PITCH)]->setTorque(pitch_torque);
    motors_[static_cast<size_t>(MotorType::YAW)]->setTorque(yaw_torque);

    command_count_++;
    if ((command_count_ % 3000U) == 0U)
    {
      XR_LOG_INFO(
          "WebotsGimbal torque cmd=%u plan=%d pitch_err=%f yaw_err=%f pitch_rate=%f yaw_rate=%f target_pitch_vel=%f target_yaw_vel=%f target_pitch_acc=%f target_yaw_acc=%f pitch_torque=%f yaw_torque=%f",
          command_count_, have_plan_ ? 1 : 0, static_cast<double>(pitch_error),
          static_cast<double>(yaw_error), static_cast<double>(pitch_rate),
          static_cast<double>(yaw_rate), static_cast<double>(target_pitch_vel),
          static_cast<double>(target_yaw_vel),
          static_cast<double>(target_pitch_acc_),
          static_cast<double>(target_yaw_acc_), static_cast<double>(pitch_torque),
          static_cast<double>(yaw_torque));
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

  webots::Motor *motors_[static_cast<size_t>(MotorType::NUMBER)]{};
  bool motors_enabled_{false};
  bool have_target_{false};
  bool have_plan_{false};
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

  LibXR::Topic gyro_topic_ = LibXR::Topic::FindOrCreate<GyroStamped>("camera_gyro");
  LibXR::Topic target_eulr_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::EulerAngle<float>>("target_eulr");
  LibXR::Topic gimbal_plan_topic_ =
      LibXR::Topic::FindOrCreate<GimbalPlan>("gimbal_plan");
  LibXR::Topic::Domain gimbal_domain_ = LibXR::Topic::Domain("gimbal");
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain_);
};
