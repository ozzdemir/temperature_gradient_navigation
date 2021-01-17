#include "ros/ros.h"
#include "chrono"
#include "temperature_gradient_navigation/temperature_gradient_navigation.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temperature_gradient_navigation_node");
  ros::NodeHandle nh("~");

  // Taking parameters
  bool use_offline_map_param;
  nh.param<bool>("use_offline_map",use_offline_map_param,true);
  // Initializing temperature_gradient_navigation object
  double hot_temperature = 1e6;
  int64_t algorithm_runtime;
  int algorithm_ret=1;
  temperature_gradient_navigation planner(nh, hot_temperature, -hot_temperature, false, true);

  // Publish temperature map
  image_transport::ImageTransport it(nh);
  image_transport::Publisher temperature_map_pub = it.advertise("temperature_map", 1, true);
  const cv::Mat *temperature_map_source = planner.get_temperaturemap_ptr();
  cv_bridge::CvImagePtr temperature_map_ptr(new cv_bridge::CvImage);
  ros::Publisher execution_time_visualization_pub = nh.advertise<visualization_msgs::Marker>("/visualization/execution_time", 1, true);
  ros::Publisher costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/temperature_costmap",1,true);
  // Periodically publish map
  ros::Timer map_publishing_timer = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &evt) {
    cv::Mat temperature_map_downscaled = cv::Mat(temperature_map_source->size(), CV_8UC1);
    cv::Mat temperature_map_downscaled_flipped = cv::Mat(temperature_map_source->size(), CV_8UC1);
    temperature_map_source->convertTo(temperature_map_downscaled, CV_8UC1, 127.5 / hot_temperature, 127.5);
    // Publishes image is reversed due to unknown reason, flip it to make it parallell to the occupancy grid.
    cv::flip(temperature_map_downscaled, temperature_map_downscaled_flipped, 0);
    auto stamp = ros::Time::now();
    temperature_map_ptr->image = temperature_map_downscaled_flipped;
    temperature_map_ptr->encoding = "mono8";
    temperature_map_ptr->header.stamp = stamp;
    temperature_map_ptr->header.frame_id = "temperature_map";
    temperature_map_pub.publish(temperature_map_ptr->toImageMsg());

    // Publish temperature costmap
    nav_msgs::OccupancyGrid costmap_msg;
    cv::Mat temporary_mat = temperature_map_downscaled * 100/255;
    costmap_msg.info = *planner.get_map_metadata();
    std::vector<int8_t> costmap_data(reinterpret_cast<int8_t*>(temporary_mat.data), reinterpret_cast<int8_t*>(temporary_mat.data + costmap_msg.info.height*costmap_msg.info.width));
    costmap_msg.data = costmap_data;
    costmap_pub.publish(costmap_msg);

    // Publish state marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = "execution_time";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = std::to_string(algorithm_runtime) + " microseconds, state: " + std::to_string(algorithm_ret);
    marker.pose.position.y = 6;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    execution_time_visualization_pub.publish(marker);
  });
  ros::Rate r(500);
  while (nh.ok())
  {
    ros::spinOnce();
    if (algorithm_ret != -1) // -1 means no path exists
    {
      auto t1 = std::chrono::high_resolution_clock::now();
      algorithm_ret = planner.iterate_algorithm();
      auto t2 = std::chrono::high_resolution_clock::now();
      algorithm_runtime = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
      //std::cout << algorithm_ret << std::endl;
    }
    //r.sleep();
  }
  return 0;
}