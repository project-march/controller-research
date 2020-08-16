#ifndef ODRIVE_INTERFACE_ODRIVE_MOTOR_H
#define ODRIVE_INTERFACE_ODRIVE_MOTOR_H

#include <utility>
#include <cmath>

#include "ros/ros.h"
#include "odrive_interface/odrive.hpp"

static constexpr double PI_2 = 2 * M_PI;
static constexpr double MOTOR_KV = 100;
static constexpr double CURRENT_TO_TORQUE_CONVERSION = 8.27;

class OdriveMotor : public Odrive
{
public:
  OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint, bool importJson = true);

  int initialize();

  int setState(uint8_t state);
  int getState();

  float getMotorControllerVoltage();
  float getMotorCurrent();
  float getTorque();

  float getAngleRadAbsolute();
  float getVelocityRadAbsolute();

  float getAngleRadIncremental();
  float getVelocityRadIncremental();

  std::string joint_name;

private:
  std::string create_command(std::string command_name);
};

#endif  // ODRIVE_INTERFACE_ODRIVE_MOTOR_H
