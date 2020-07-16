#include "odrive_interface/odrive.hpp"
#include "odrive_interface/odrive_endpoint.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odrive_test_node");
  ros::NodeHandle nh;

  ros::Rate r(1);

  ROS_INFO("Starting odrives...");

  std::string base_path_odrives = ros::package::getPath("odrive_interface");
  base_path_odrives.append("/config/odrives.yaml");

  YAML::Node odrives_configuration(YAML::LoadFile(base_path_odrives));

  std::string serial_number = odrives_configuration.begin()->first.as<std::string>();

  auto* odrive_endpoint = new OdriveEndpoint();
  std::vector<Odrive> odrives_objects;

  for (const YAML::Node& joint : odrives_configuration[serial_number])
  {
    std::string joint_name = joint.begin()->first.as<std::string>();
    std::string axis_name = joint[joint_name]["axis"].as<std::string>();

    ROS_INFO("Init %s with sn; %s and axis nr; %s", joint_name.c_str(), serial_number.c_str(), axis_name.c_str());
    Odrive odrive(joint_name, axis_name, odrive_endpoint);
    odrives_objects.push_back(odrive);
  }

  odrive_endpoint->open_connection(serial_number);

}