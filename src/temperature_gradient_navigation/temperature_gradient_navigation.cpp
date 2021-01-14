#include "temperature_gradient_navigation/temperature_gradient_navigation.h"
#include "tf/tf.h"
temperature_gradient_navigation::temperature_gradient_navigation(ros::NodeHandle &nh, double hot_temperature, double cold_temperature, bool visualization)
{
    nh_ = nh;
    hot_temperature_ = hot_temperature;
    cold_temperature_ = cold_temperature;
    //controller_timer_ = nh_.createTimer(ros::Duration(0.01), &temperature_gradient_navigation::controller_cb, this);

    odom_subs_ = nh_.subscribe("/odom", 1, &temperature_gradient_navigation::odom_cb, this);
    map_subs_ = nh_.subscribe("/map", 1, &temperature_gradient_navigation::map_cb, this);
    goalpose_subs_ = nh_.subscribe("/move_base_simple/goal", 1, &temperature_gradient_navigation::goalpose_cb, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, this);
    gz_sms_cli_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    visualization_ = visualization;
    //temperature_map_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/neighborhoods_marker", 1, true);
}

void temperature_gradient_navigation::odom_cb(const nav_msgs::Odometry &msg)
{
    position_ = msg.pose.pose.position;
}

void temperature_gradient_navigation::map_cb(const nav_msgs::OccupancyGridConstPtr &msg)
{
    static bool temperature_map_initialized = false;
    map_metadata_ = msg->info;
    int size_x = msg->info.width;
    int size_y = msg->info.height;
    cv::Mat temp_map = cv::Mat(size_x, size_y, CV_8S, (void *)msg->data.data());
    auto unknowns_mask = temp_map == -1;
    auto occupied_mask = (temp_map > 50) & (~unknowns_mask);
    auto empty_mask = (temp_map < 10) & (~unknowns_mask);

    temp_map.convertTo(temp_map, CV_8U);
    temp_map.setTo(255, empty_mask);
    temp_map.setTo(0, occupied_mask);
    if (!temperature_map_initialized)
    {
        temperature_map_initialized = true;
        temperature_map_ = cv::Mat(size_x, size_y, CV_64F, hot_temperature_);
    }
    set_tf_mats(map_metadata_); // Set coordinate transform matrices
    //ROS_INFO("First one %d, %lf",temp_map.channels(), map_yaw);
    //cv::imshow("sda", temp_map);
    //cv::waitKey(0);
}

void temperature_gradient_navigation::controller_cb(const ros::TimerEvent &evt)
{
    static cv::Vec2i cur_pt;
    static cv::Vec2i old_pt;
    static cv::Mat_<double> pos(3, 1);
    static cv::Mat_<double> pixel_pos(3, 1);
    double sinx, cosx;
    static geometry_msgs::Twist cmd_vel_msg;
    static double dist;

    pos(0) = position_.x;
    pos(1) = position_.y;
    pos(2) = 1.0;
    pixel_pos = (real2pixel_tf_mat_ * pos);
    cur_pt[0] = int(pixel_pos(0));
    cur_pt[1] = int(pixel_pos(1));
    dist = calc_distance(cur_pt, goal_position_);
    double angle = get_gradient_angle(cur_pt); //TODO: When agent goes out of scope, this results segfault
    double mag = get_gradient_magnitude(cur_pt);
    sincos(map_yaw_angle_ - angle, &sinx, &cosx); // Angles in gazebo and image are not same, thats why substracting from pi/2
    old_pt = cur_pt;
    cmd_vel_msg.linear.x = -0.01 * dist * cosx; // Magnitude of gradient becomes so large at boundaries, thats why using another one.
    cmd_vel_msg.linear.y = -0.01 * dist * sinx;
    cmd_vel_pub_.publish(cmd_vel_msg);
}

void temperature_gradient_navigation::initialpose_cb(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    gz_set_position(msg.pose.pose.position.x, msg.pose.pose.position.y);
}

void temperature_gradient_navigation::goalpose_cb(const geometry_msgs::PoseStamped &msg)
{
    static cv::Vec2i goal_pt;
    static cv::Mat_<double> goal_pos(3, 1);
    static cv::Mat_<double> pixel_pos(3, 1);
    goal_pos(0) = msg.pose.position.x;
    goal_pos(1) = msg.pose.position.y;
    goal_pos(2) = 1;
    pixel_pos = real2pixel_tf_mat_ * goal_pos;
    goal_position_(0) = int(pixel_pos(0));
    goal_position_(1) = int(pixel_pos(1));
    goal_initialized_ = true;
    // Re-initialize temperature map
    temperature_map_.setTo(hot_temperature_);
    std::cout << goal_position_ << std::endl;
    temperature_map_.at<double>(goal_position_(1), goal_position_(0)) = cold_temperature_;
}

void temperature_gradient_navigation::set_tf_mats(nav_msgs::MapMetaData metadata)
{ // TODO: set matrices such that it is universal?
    double pixel2real_mat_elements[9] = {map_metadata_.resolution, 0, -map_metadata_.origin.position.x,
                                         0, -map_metadata_.resolution, map_metadata_.origin.position.y,
                                         0, 0, 1};
    double real2pixel_mat_elements[9] = {1 / map_metadata_.resolution, 0,  -map_metadata_.origin.position.x / map_metadata_.resolution,
                                         0, 1 / map_metadata_.resolution, -map_metadata_.origin.position.y / map_metadata_.resolution,
                                         0, 0, 1};
    pixel2real_tf_mat_ = cv::Mat(3, 3, CV_64F, pixel2real_mat_elements).clone();
    real2pixel_tf_mat_ = cv::Mat(3, 3, CV_64F, real2pixel_mat_elements).clone();
    map_yaw_angle_ = tf::getYaw(map_metadata_.origin.orientation);
}

void temperature_gradient_navigation::update_anglemap()
{
    cv::Mat dx, dy;
    cv::Sobel(temperature_map_, dx, CV_64F, 1, 0, 1);
    cv::Sobel(temperature_map_, dy, CV_64F, 0, 1, 1);
    cv::cartToPolar(dx, dy, magnitudemap_, anglemap_); // Anglemap later can be used in traversal
}

void temperature_gradient_navigation::temperature_iterator()
{
    temperature_map_.forEach<double>([&](double &p, const int *position) -> void {
        if (p != INT_MAX) // Meaning that pixel is not on object
        {
        }
        else
        {
            p = INT_MAX;
        }
    });
}

double temperature_gradient_navigation::get_gradient_angle(cv::Vec2i q)
{
    return anglemap_.at<double>(q[1], q[0]);
}

double temperature_gradient_navigation::get_gradient_magnitude(cv::Vec2i q)
{
    return magnitudemap_.at<double>(q[1], q[0]);
}

double temperature_gradient_navigation::calc_distance(cv::Vec2i q1, cv::Vec2i q2)
{
    double distance = cv::norm(q1 - q2);
    return distance;
}

double temperature_gradient_navigation::calc_distance_sqrd(cv::Vec2i q1, cv::Vec2i q2)
{
    double tmp = calc_distance(q1, q2);
    return tmp * tmp;
}

bool temperature_gradient_navigation::gz_set_position(double x, double y)
{
    static gazebo_msgs::SetModelState sms_msg;
    sms_msg.request.model_state.model_name = "agent";
    sms_msg.request.model_state.pose.position.x = x;
    sms_msg.request.model_state.pose.position.y = y;
    sms_msg.request.model_state.reference_frame = "map";
    bool ret = gz_sms_cli_.call(sms_msg);
    return ret;
}