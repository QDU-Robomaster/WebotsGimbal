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

#include <cmath>

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

 public:
  WebotsGimbal(LibXR::HardwareContainer &, LibXR::ApplicationManager &app)
  {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");

    for (size_t i = 0; i < static_cast<size_t>(MotorType::NUMBER); i++)
    {
      motors_[i] = _libxr_webots_robot_handle->getMotor(MOTOR_NAMES[i]);
      motors_[i]->setVelocity(10.0f);
      motors_[i]->setForce(10.0f);
      motors_[i]->setPosition(0.0f);
    }

    auto cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsGimbal *self, LibXR::RawData &data)
        {
          auto eulr = *static_cast<LibXR::EulerAngle<float> *>(data.addr_);
          if (!std::isfinite(eulr.Pitch()) || !std::isfinite(eulr.Yaw()))
          {
            XR_LOG_WARN("WebotsGimbal ignored non-finite target_eulr pitch=%f yaw=%f",
                        static_cast<double>(eulr.Pitch()),
                        static_cast<double>(eulr.Yaw()));
            return;
          }
          self->motors_[static_cast<size_t>(MotorType::PITCH)]->setPosition(eulr.Pitch());
          self->motors_[static_cast<size_t>(MotorType::YAW)]->setPosition(eulr.Yaw());
        },
        this);

    target_eulr_topic_.RegisterCallback(cb);

    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
  webots::Motor *motors_[static_cast<size_t>(MotorType::NUMBER)];

  // Keep a default-domain topic handle alive so SharedTopic_MCU can find it,
  // while actual motor commands subscribe directly to the tracker-domain topic.
  LibXR::Topic bridge_target_eulr_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::EulerAngle<float>>("target_eulr");
  LibXR::Topic::Domain tracker_domain_ = LibXR::Topic::Domain("tracker");
  LibXR::Topic target_eulr_topic_ =
      LibXR::Topic("target_eulr", sizeof(LibXR::EulerAngle<float>), &tracker_domain_);
};
