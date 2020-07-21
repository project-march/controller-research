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

  std::string base_path_odrives = ros::package::getPath("odrive_interface");
  base_path_odrives.append("/config/odrives/odrives.yaml");

  YAML::Node odrives = YAML::LoadFile(base_path_odrives);
  const auto serial_number = odrives.begin()->first.as<std::string>();

  auto* odrive_endpoint = new OdriveEndpoint();
  std::vector<Odrive> odrives_objects;

  const YAML::Node& odrives_configuration = odrives[serial_number];
  for (const YAML::Node& odrive_joint : odrives_configuration)
  {
    ROS_INFO("odrive motor %s", serial_number.c_str());

    std::string joint_name = odrive_joint.begin()->first.as<std::string>();
    std::string axis_name = odrive_joint[joint_name]["axis"].as<std::string>();

    ROS_INFO("Init %s with sn; %s and axis nr; %s", joint_name.c_str(), serial_number.c_str(), axis_name.c_str());
    Odrive odrive(joint_name, axis_name, odrive_endpoint);
    odrives_objects.push_back(odrive);
  }

  odrive_endpoint->open_connection(serial_number);

  ros::Publisher odrive_publisher = nh.advertise<odrive_interface::odrive_msg>("odrive_msg_" + serial_number, 100);

  ROS_INFO("Starting idle loop");
  while (ros::ok())
  {
    // Publish status message
    publishOdriveData(odrive_publisher, odrives_objects[0]);

    // update watchdog
    odrives_objects[0].function("axis0.watchdog_feed");
    odrives_objects[0].function("axis1.watchdog_feed");

    r.sleep();
    ros::spinOnce();
  }
}
