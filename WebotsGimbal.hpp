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

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "app_framework.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "transform.hpp"

extern webots::Robot *_libxr_webots_robot_handle;

class WebotsGimbal : public LibXR::Application
{
  enum class MotorType
  {
    PITCH,
    YAW,
    NUMBER
  };

  static constexpr const char *MOTOR_NAMES[2] = {"target_motor_pitch",
                                                 "target_motor_yaw"};
  static constexpr float PI = 3.14159265358979323846f;

  static float LimitRad(float angle)
  {
    while (angle > PI)
    {
      angle -= 2.0f * PI;
    }
    while (angle <= -PI)
    {
      angle += 2.0f * PI;
    }
    return angle;
  }

 public:
  WebotsGimbal(LibXR::HardwareContainer &, LibXR::ApplicationManager &app)
  {
    const char *disable_env = std::getenv("XR_DISABLE_GIMBAL_CONTROL");
    control_disabled_ =
        disable_env != nullptr && disable_env[0] != '\0' && disable_env[0] != '0';
    const char *pitch_slew_env = std::getenv("XR_WEBOTS_PITCH_SLEW_STEP_RAD");
    if (pitch_slew_env != nullptr && pitch_slew_env[0] != '\0')
    {
      pitch_slew_step_rad_ = std::max(0.0f, std::strtof(pitch_slew_env, nullptr));
    }
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");

    for (size_t i = 0; i < static_cast<size_t>(MotorType::NUMBER); i++)
    {
      motors_[i] = _libxr_webots_robot_handle->getMotor(MOTOR_NAMES[i]);
      motors_[i]->setVelocity(10.0f);
      motors_[i]->setForce(10.0f);
      motors_[i]->setPosition(0.0f);
    }

    if (control_disabled_)
    {
      XR_LOG_WARN("WebotsGimbal control disabled by XR_DISABLE_GIMBAL_CONTROL");
    }
    if (pitch_slew_step_rad_ > 0.0f)
    {
      XR_LOG_WARN("WebotsGimbal pitch slew enabled step_rad=%f",
                  static_cast<double>(pitch_slew_step_rad_));
    }

    auto cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          if (self->control_disabled_)
          {
            return;
          }
          auto eulr = *static_cast<LibXR::EulerAngle<float> *>(data.addr_);
          if (!std::isfinite(eulr.Pitch()) || !std::isfinite(eulr.Yaw()))
          {
            XR_LOG_WARN("WebotsGimbal ignored non-finite target_eulr pitch=%f yaw=%f",
                        static_cast<double>(eulr.Pitch()),
                        static_cast<double>(eulr.Yaw()));
            return;
          }
          float pitch = eulr.Pitch();
          if (self->pitch_slew_step_rad_ > 0.0f)
          {
            if (self->has_last_pitch_command_)
            {
              pitch = std::clamp(
                  pitch,
                  self->last_pitch_command_ - self->pitch_slew_step_rad_,
                  self->last_pitch_command_ + self->pitch_slew_step_rad_);
            }
            self->last_pitch_command_ = pitch;
            self->has_last_pitch_command_ = true;
          }
          self->motors_[static_cast<size_t>(MotorType::PITCH)]->setPosition(
              -pitch);
          self->motors_[static_cast<size_t>(MotorType::YAW)]->setPosition(
              LimitRad(eulr.Yaw() + 0.5f * PI));
        },
        this);

    target_eulr_topic_.RegisterCallback(cb);

    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
  webots::Motor *motors_[static_cast<size_t>(MotorType::NUMBER)];
  bool control_disabled_{false};
  float pitch_slew_step_rad_{0.0f};
  float last_pitch_command_{0.0f};
  bool has_last_pitch_command_{false};

  LibXR::Topic target_eulr_topic_ =
      LibXR::Topic("target_eulr", sizeof(LibXR::EulerAngle<float>));
};
