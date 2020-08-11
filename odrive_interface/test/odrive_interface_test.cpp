#include "odrive_interface/odrive_msg.h"
#include "odrive_interface/odrive.hpp"
#include "odrive_interface/odrive_endpoint.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <yaml-cpp/yaml.h>

void publishOdriveData(const ros::Publisher& odrive_publisher, Odrive odrive_object)
{
  odrive_interface::odrive_msg msg;

  float v_bus;
  odrive_object.read("vbus_voltage", v_bus);
  msg.vbus = v_bus;

  odrive_publisher.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odrive_test_node");
  ros::NodeHandle nh;

  ros::Rate r(1);

  ROS_INFO("Starting odrives...");

  std::string path_odrives_configuration = ros::package::getPath("odrive_interface");
  std::string path_odrive_setting = ros::package::getPath("odrive_interface");

  path_odrives_configuration.append("/config/odrives/odrives.yaml");
  path_odrive_setting.append("/config/odrive_settings/march_odrive.json");

  YAML::Node odrives = YAML::LoadFile(path_odrives_configuration);
  const auto serial_number = odrives.begin()->first.as<std::string>();

  std::shared_ptr<OdriveEndpoint> odrive_endpoint = std::make_shared<OdriveEndpoint>();
  std::vector<Odrive> odrives_objects;

  ROS_INFO("Open serial connection");
  odrive_endpoint->open_connection(serial_number);

  const YAML::Node& odrives_configuration = odrives[serial_number];
  for (const YAML::Node& odrive_joint : odrives_configuration)
  {
    ROS_INFO("odrive motor %s", serial_number.c_str());

    std::string joint_name = odrive_joint.begin()->first.as<std::string>();
    std::string axis_name = odrive_joint[joint_name]["axis"].as<std::string>();

    ROS_INFO("Init %s with sn; %s and axis nr; %s", joint_name.c_str(), serial_number.c_str(), axis_name.c_str());
    Odrive odrive(joint_name, axis_name, odrive_endpoint);

    ROS_INFO("Configure the odrive with file %s", path_odrive_setting.c_str());
    odrive.setConfigurations(path_odrive_setting);

    odrives_objects.push_back(odrive);
  }

  ros::Publisher odrive_publisher_1 =
      nh.advertise<odrive_interface::odrive_msg>("odrive_msg_" + odrives_objects[0].axis_number, 100);
  ros::Publisher odrive_publisher_2 =
      nh.advertise<odrive_interface::odrive_msg>("odrive_msg_" + odrives_objects[1].axis_number, 100);

  ROS_INFO("Starting idle loop");
  while (ros::ok())
  {
    // Publish status message
    publishOdriveData(odrive_publisher_1, odrives_objects[0]);
    publishOdriveData(odrive_publisher_2, odrives_objects[1]);

    // update watchdog
    odrives_objects[0].function("axis0.watchdog_feed");
    odrives_objects[1].function("axis1.watchdog_feed");

    r.sleep();
    ros::spinOnce();
  }
}
