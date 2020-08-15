#ifndef ODRIVE_INTERFACE_ODRIVE_MOTOR_H
#define ODRIVE_INTERFACE_ODRIVE_MOTOR_H

#include <utility>

#include "ros/ros.h"
#include "odrive_interface/odrive.hpp"
#include "odrive_interface/odrive_enums.hpp"

class OdriveMotor : public Odrive
{
public:
  OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint, std::string& joint_name,
              bool importJson = true);

  int get_position();

  float get_input_voltage();

  int get_axis_error();

  std::string joint_name;
};

#endif  // ODRIVE_INTERFACE_ODRIVE_MOTOR_H
