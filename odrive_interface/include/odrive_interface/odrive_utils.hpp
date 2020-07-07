#ifndef ODRIVE_UTILS_HPP_
#define ODRIVE_UTILS_HPP_

#include <unistd.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>

#include "ros/ros.h"
#include "odrive_interface/odrive.hpp"
#include "odrive_interface/odrive_endpoint.hpp"
typedef struct _odrive_object
{
  int id;

  std::string name;
  std::string type;
  std::string access;
} odrive_object;

int getJson(odrive_endpoint* endpoint, Json::Value* json);
int setChannelConfig(odrive_endpoint* endpoint, const Json::Value& odrive_json, Json::Value config_json, bool save_config);

int updateTargetConfig(odrive_endpoint* endpoint, const Json::Value& odrive_json, const std::string& config_file);
int calibrateAxis0(odrive_endpoint* endpoint, const Json::Value& odrive_json);

int getObjectByName(Json::Value odrive_json, std::string name, odrive_object* odo);

template <typename TT>
int readOdriveData(odrive_endpoint* endpoint, Json::Value odrive_json, std::string object, TT& value);
template <typename T>
int writeOdriveData(odrive_endpoint* endpoint, Json::Value odrive_json, std::string object, T& value);

int execOdriveFunc(odrive_endpoint* endpoint, Json::Value odrive_json, std::string object);
int execOdriveGetTemp(odrive_endpoint* endpoint, Json::Value odrive_json, std::string object, float& temp);

#endif
