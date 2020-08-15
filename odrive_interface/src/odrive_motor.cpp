#include "odrive_interface/odrive_motor.h"

#include <utility>

OdriveMotor::OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint,
                         std::string& joint_name, bool importJson)
  : Odrive(axisNumber, std::move(odriveEndpoint), importJson)
{
  this->joint_name = joint_name;
}

int OdriveMotor::get_position()
{
  int position;
  std::string parameter_name = this->axis_number + PM_ENCODER_COUNTS;

  this->read(parameter_name, position);
  return position;
}

float OdriveMotor::get_input_voltage()
{
  float odrive_input_voltage;

  this->read(ODRIVE_INPUT_VOLTAGE, odrive_input_voltage);
  return odrive_input_voltage;
}
