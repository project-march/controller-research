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

typedef struct odrive_json_object
{
  int id;
  std::string name;
  std::string type;
  std::string access;
} odrive_json_object;

class Odrive
{
public:
  /**
   * Initialize the odrive with specified axis
   */
  Odrive(const std::string& joint_name, const std::string& axis_number, OdriveEndpoint* odrive_endpoint);

  /**
   * Check if given value type matched value type of odrive variable
   */
  template <typename TT>
  int validateType(const odrive_json_object& json_object, TT& value);

  /**
   * Read parameter from the odrive object
   */
  template <typename TT>
  int read(const std::string& parameter_name, TT& value);

  /**
   * Write parameter to the odrive object
   */
  template <typename TT>
  int write(const std::string& parameter_name, TT& value);

  /**
   * Execute function on the odrive object
   */
  int function(const std::string& function_name);

  /**
   * Set configurations in the Json file to the Odrive
   */
  int setConfigurations(const std::string& configuration_json_path);

  std::string joint_name;
  std::string axis_number;

private:
  /**
  * Get the Json configuration structure from the odrive
  */
  int getJson();

  /**
  * Split given odrive parameter name with standard delimiter .
  */
  std::vector<std::string> split_string(const std::string& str, char delimiter = '.');

  /**
  * Set configurations in the Json file to the Odrive
  */
  odrive_json_object getJsonObject(const std::string& parameter_name);

  Json::Value odrive_json_;
  Json::Value odrive_configuration_json_;

  OdriveEndpoint* odrive_endpoint_;
};
#endif
