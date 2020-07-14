#ifndef ODRIVE_HPP_
#define ODRIVE_HPP_

#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <string>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "ros/ros.h"
#include "odrive_interface/odrive_endpoint.hpp"
#include "odrive_interface/odrive_enums.hpp"
#include "odrive_interface/odrive_msg.h"

class Odrive
{
public:
  /**
   * initialize the odrive with specified axis
   */
  Odrive(const std::string& serial_number, const std::string& axis_number);

  /**
   * Destroy the odrive object
   */
  ~Odrive();

  std::string serial_number;
  std::string axis_number;

private:
  int getJson();

  Json::Value odrive_json_;
  OdriveEndpoint odrive_endpoint_;
};
#endif
