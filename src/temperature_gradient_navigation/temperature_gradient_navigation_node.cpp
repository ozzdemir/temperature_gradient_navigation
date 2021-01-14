#include "ros/ros.h"
#include "temperature_gradient_navigation/temperature_gradient_navigation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temperature_gradient_navigation_node");
  ros::NodeHandle nh("~");

  // Taking parameters

  // Initializing temperature_gradient_navigation object
  temperature_gradient_navigation planner(nh,1e36, -1e36, true);

  while (nh.ok())
  {
    ros::spin();
  }
  return 0;
}