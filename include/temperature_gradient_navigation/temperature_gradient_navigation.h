#ifndef TEMPERATURE_GRADIENT_NAVIGATION_H_
#define TEMPERATURE_GRADIENT_NAVIGATION_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"

#include "opencv2/opencv.hpp"

enum algorithm_state
{
    state_no_path = -1,
    state_goal_achieved = 0,
    state_initializing = 1,
    state_traversing = 2
};

class temperature_gradient_navigation
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_subs_;
    ros::Subscriber goalpose_subs_;
    ros::Subscriber initialpose_subs_;
    ros::Subscriber map_subs_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher neighborhoods_marker_pub_;
    ros::ServiceClient gz_sms_cli_;
    ros::ServiceClient mapserver_cli_;
    ros::Timer controller_timer_;
    ros::Timer gradient_updating_timer_;

    geometry_msgs::Point position_;
    nav_msgs::MapMetaData map_metadata_;

    cv::Mat map_, temperature_map_, temperature_map_prev_, anglemap_, map_bw_, magnitudemap_;
    cv::Mat pixel2real_tf_mat_, real2pixel_tf_mat_;
    cv::Vec2i goal_position_;
    cv::Vec2i start_position_;

    double cold_temperature_, hot_temperature_, map_yaw_angle_;
    bool visualization_;
    bool goal_initialized_, start_initialized_, algorithm_initialized_;

    void odom_cb(const nav_msgs::Odometry &msg);
    void map_cb(const nav_msgs::OccupancyGrid &msg);
    void map_metadata_cb(const nav_msgs::MapMetaData &msg);
    void controller_cb(const ros::TimerEvent &evt);
    void initialpose_cb(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void goalpose_cb(const geometry_msgs::PoseStamped &msg);
    void update_gradient(const ros::TimerEvent &evt);

    void set_tf_mats(nav_msgs::MapMetaData metadata);
    void update_temperatures();
    double get_gradient_angle(cv::Vec2i q);
    double get_gradient_magnitude(cv::Vec2i q);
    double calc_distance(cv::Vec2i q1, cv::Vec2i q2);
    double calc_distance_sqrd(cv::Vec2i q1, cv::Vec2i q2);
    bool gz_set_position(double x, double y);
    double max_diff();
    double get_temperature(cv::Vec2i q);
    void clip(double &n, double lower, double upper);

public:
    temperature_gradient_navigation(ros::NodeHandle &nh, double hot_temperature, double cold_temperature, bool use_offline_map = true, bool visualization = true);
    int iterate_algorithm();
    const nav_msgs::MapMetaData *get_map_metadata();
    const cv::Mat *get_map_ptr();
    const cv::Mat *get_temperaturemap_ptr();
    const cv::Mat *get_anglemap_ptr();
};
#endif // TEMPERATURE_GRADIENT_NAVIGATION_H_
