#include "ros/ros.h"
#include "temperature_gradient_navigation/temperature_gradient_navigation.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temperature_gradient_navigation_node");
  ros::NodeHandle nh("~");

  // Taking parameters

  // Initializing temperature_gradient_navigation object
  double hot_temperature = 1e6;
  temperature_gradient_navigation planner(nh, hot_temperature, -hot_temperature, true);
  // Publish temperature map

  image_transport::ImageTransport it(nh);
  image_transport::Publisher temperature_map_pub = it.advertise("temperature_map", 1, true);
  const cv::Mat *temperature_map_source = planner.get_temperaturemap_ptr();
  cv_bridge::CvImagePtr temperature_map_ptr(new cv_bridge::CvImage);

  // Periodically publish map
  ros::Timer map_publishing_timer = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &evt) {
    cv::Mat temperature_map_downscaled = cv::Mat(temperature_map_source->size(), CV_8UC1);
    cv::Mat temperature_map_downscaled_flipped = cv::Mat(temperature_map_source->size(), CV_8UC1);
    temperature_map_source->convertTo(temperature_map_downscaled, CV_8UC1, 127.5 / hot_temperature, 127.5);
    // Publishes image is reversed due to unknown reason, flip it to make it parallell to the occupancy grid.
    cv::flip(temperature_map_downscaled, temperature_map_downscaled_flipped, 0); 
    temperature_map_ptr->image = temperature_map_downscaled_flipped;
    temperature_map_ptr->encoding = "mono8";
    temperature_map_ptr->header.stamp = ros::Time::now();
    temperature_map_ptr->header.frame_id = "temperature_map";
    temperature_map_pub.publish(temperature_map_ptr->toImageMsg());
    //std::cout << temperature_map_source->at<double>(100, 100) << std::endl;
  });
  while (nh.ok())
  {
    ros::spin();
  }
  return 0;
}