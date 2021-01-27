#include "temperature_gradient_navigation/temperature_gradient_navigation.h"
#include "tf/tf.h"
temperature_gradient_navigation::temperature_gradient_navigation(ros::NodeHandle &nh, double hot_temperature, double cold_temperature, bool use_offline_map, bool visualization)
{
    nh_ = nh;
    hot_temperature_ = hot_temperature;
    cold_temperature_ = cold_temperature;
    controller_timer_ = nh_.createTimer(ros::Duration(0.01), &temperature_gradient_navigation::controller_cb, this);
    gradient_updating_timer_ = nh_.createTimer(ros::Duration(0.1), &temperature_gradient_navigation::update_gradient, this);

    odom_subs_ = nh_.subscribe("/odom", 1, &temperature_gradient_navigation::odom_cb, this);
    goalpose_subs_ = nh_.subscribe("/move_base_simple/goal", 1, &temperature_gradient_navigation::goalpose_cb, this);
    initialpose_subs_ = nh_.subscribe("/initialpose", 1, &temperature_gradient_navigation::initialpose_cb, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    gz_sms_cli_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    visualization_ = visualization;

    if (use_offline_map)
    {
        static nav_msgs::GetMap payload;

        mapserver_cli_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");

        if (mapserver_cli_.waitForExistence(ros::Duration(5)))
        {
            mapserver_cli_.call(payload);
            map_cb(payload.response.map);
            std::cout << payload.response.map.info.origin;
        }
        else
        {
            ROS_ERROR("No map found in map server. Exiting.");
        }
    }
    else
    {
        map_subs_ = nh_.subscribe("/map", 1, &temperature_gradient_navigation::map_cb, this);
    }

    //temperature_map_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/neighborhoods_marker", 1, true);
}

void temperature_gradient_navigation::odom_cb(const nav_msgs::Odometry &msg)
{
    position_ = msg.pose.pose.position;
}

void temperature_gradient_navigation::map_cb(const nav_msgs::OccupancyGrid &msg)
{
    static bool temperature_map_initialized = false;
    int size_x = msg.info.width;
    int size_y = msg.info.height;
    cv::Mat map = cv::Mat(size_y, size_x, CV_8S, (void *)msg.data.data());
    map.setTo(25, (map == -1));
    map.convertTo(map, CV_8U);
    map.setTo(255, (map > 50));
    map.copyTo(map_);
    if (!temperature_map_initialized)
    {
        map_metadata_ = msg.info;
        //std::cout << map_ << std::endl;
        temperature_map_initialized = true;
        temperature_map_ = cv::Mat(size_y, size_x, CV_64F, hot_temperature_);
        //temperature_map_.setTo(0, map > 50);
        anglemap_ = cv::Mat(size_y, size_x, CV_64F, 1e-36);
        magnitudemap_ = cv::Mat(size_y, size_x, CV_64F, 1e-36);
    }
    set_tf_mats(map_metadata_); // Set coordinate transform matrices
}

void temperature_gradient_navigation::controller_cb(const ros::TimerEvent &evt)
{
    static cv::Vec2i cur_pt;
    static cv::Mat_<double> pos(3, 1);
    static cv::Mat_<double> pixel_pos(3, 1);
    double sinx, cosx;
    static geometry_msgs::Twist cmd_vel_msg;
    static double dist;
    if (algorithm_initialized_)
    {
        pos(0) = position_.x;
        pos(1) = position_.y;
        pos(2) = 1.0;
        pixel_pos = (real2pixel_tf_mat_ * pos);
        cur_pt[0] = int(pixel_pos(0));
        cur_pt[1] = int(pixel_pos(1));
        if (get_temperature(cur_pt) < hot_temperature_)
        {
            dist = calc_distance(cur_pt, goal_position_);
            double angle = get_gradient_angle(cur_pt); //TODO: When agent goes out of scope, this results segfault
            double mag = get_gradient_magnitude(cur_pt);

            sincos(angle, &sinx, &cosx);

            double base_velocity = dist;
            clip(base_velocity, 0, 0.5);
            cmd_vel_msg.linear.x = -base_velocity * cosx;
            cmd_vel_msg.linear.y = -base_velocity * sinx;
            cmd_vel_pub_.publish(cmd_vel_msg);
        }
        else
        {
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.linear.y = 0;
            cmd_vel_pub_.publish(cmd_vel_msg);
        }
    }
}

void temperature_gradient_navigation::initialpose_cb(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    gz_set_position(msg.pose.pose.position.x, msg.pose.pose.position.y);
    static cv::Mat_<double> initial_pos(3, 1);
    static cv::Mat_<double> pixel_pos(3, 1);
    initial_pos(0) = msg.pose.pose.position.x;
    initial_pos(1) = msg.pose.pose.position.y;
    initial_pos(2) = 1;
    pixel_pos = real2pixel_tf_mat_ * initial_pos;
    start_position_(0) = int(pixel_pos(0));
    start_position_(1) = int(pixel_pos(1));
    start_initialized_ = true;
    std::cout << temperature_map_.at<double>(int(pixel_pos(1)), int(pixel_pos(0))) << std::endl;
    traverse_ideal(start_position_);
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
    algorithm_initialized_ = false;
    std::cout << goal_position_ << std::endl;
    temperature_map_.at<double>(goal_position_(1), goal_position_(0)) = cold_temperature_;
    std::cout << temperature_map_.at<double>(goal_position_(1), goal_position_(0)) << std::endl;
}

void temperature_gradient_navigation::set_tf_mats(nav_msgs::MapMetaData metadata)
{ // TODO: set matrices such that it is universal?
    double pixel2real_mat_elements[9] = {map_metadata_.resolution, 0, map_metadata_.origin.position.x,
                                         0, map_metadata_.resolution, map_metadata_.origin.position.y,
                                         0, 0, 1};
    double real2pixel_mat_elements[9] = {1 / map_metadata_.resolution, 0, -map_metadata_.origin.position.x / map_metadata_.resolution,
                                         0, 1 / map_metadata_.resolution, -map_metadata_.origin.position.y / map_metadata_.resolution,
                                         0, 0, 1};
    pixel2real_tf_mat_ = cv::Mat(3, 3, CV_64F, pixel2real_mat_elements).clone();
    real2pixel_tf_mat_ = cv::Mat(3, 3, CV_64F, real2pixel_mat_elements).clone();
    map_yaw_angle_ = tf::getYaw(map_metadata_.origin.orientation);
}

void temperature_gradient_navigation::update_gradient(const ros::TimerEvent &evt)
{
    if (algorithm_initialized_)
    {
        cv::Mat dx, dy;
        cv::Sobel(temperature_map_, dx, CV_64F, 1, 0, 1);
        cv::Sobel(temperature_map_, dy, CV_64F, 0, 1, 1);
        cv::cartToPolar(dx, dy, magnitudemap_, anglemap_); // Anglemap later can be used in traversal
    }
}

void temperature_gradient_navigation::update_temperatures()
{
    temperature_map_.copyTo(temperature_map_prev_);
    temperature_map_.forEach<double>([&](double &p, const int *px) -> void {
        double val = map_.at<unsigned char>(px[0], px[1]);
        if ((px[0] == goal_position_(1)) && (px[1] == goal_position_(0)))
        {
        }
        else if (px[0] == 0 | px[0] == map_metadata_.height - 1)
        {
        }
        else if (px[1] == 0 | px[1] == map_metadata_.width - 1)
        {
        }
        else
        {
            if (val != 255) // Meaning that pixel is not on object
            {
                p = (temperature_map_.at<double>(px[0] + 1, px[1]) +
                     temperature_map_.at<double>(px[0] - 1, px[1]) +
                     temperature_map_.at<double>(px[0], px[1] + 1) +
                     temperature_map_.at<double>(px[0], px[1] - 1)) /
                    4.0;
            }
            else
            {
                p = hot_temperature_;
            }
        }
    });
}

int temperature_gradient_navigation::iterate_algorithm()
{
    static double epsilon = 30;
    static int N = 1000;
    if (goal_initialized_)
    {
        if (!algorithm_initialized_)
        {
            static int count = 0;
            if (epsilon > max_diff())
            {
                update_temperatures();
                count++;
                if ((count >= N) && (get_temperature(start_position_) == hot_temperature_))
                {
                    return state_no_path;
                }
                return state_initializing;
            }
            else
            {
                algorithm_initialized_ = true;
                return state_traversing;
            }
        }
        else // After initialization
        {
            update_temperatures();
            return state_traversing;
        }
    }
    else
    {
        return state_initializing;
    }
}

bool temperature_gradient_navigation::traverse_ideal(cv::Vec2i qstart)
{
    static ros::Publisher trajectory_marker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization/trajecory_ideal", 1, true);
    cv::Vec2d cur_pos;
    cur_pos(0) = qstart(0);
    cur_pos(1) = qstart(1);
    cv::Vec2i cur_pt = qstart;
    cv::Vec2i old_pt = cur_pt;
    std::vector<cv::Vec2d> trajectory;
    double sinx, cosx;
    bool success;

    try
    {
        for (int i = 0; i < 1000; i++)
        {
            trajectory.push_back(cur_pos);
            double angle = get_gradient_angle(cur_pt);
            sincos(angle, &sinx, &cosx);
            old_pt = cur_pt;

            cur_pos[0] -= 0.5 * cosx;
            cur_pos[1] -= 0.5 * sinx;

            cur_pt[0] = int(cur_pos[0]);
            cur_pt[1] = int(cur_pos[1]);
            if (calc_distance(cur_pt, goal_position_) < 2 * map_metadata_.resolution)
            {
                success = true;
                break;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    // Publish state marker
    visualization_msgs::Marker marker;
    geometry_msgs::Point tmp_pt;
    static cv::Mat_<double> pos(3, 1);
    static cv::Mat_<double> pixel_pos(3, 1);

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_ideal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -0.05;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    for (auto i = trajectory.begin(); i != trajectory.end(); ++i)
    {
        pixel_pos(0) = (*i)(0);
        pixel_pos(1) = (*i)(1);
        pixel_pos(2) = 1.0;
        pos = (pixel2real_tf_mat_ * pixel_pos);
        tmp_pt.x = pos(0);
        tmp_pt.y = pos(1);
        marker.points.push_back(tmp_pt);
        std::cout << *i << std::endl;
    }
    trajectory_marker_pub.publish(marker);
    return success;
}

const nav_msgs::MapMetaData *temperature_gradient_navigation::get_map_metadata()
{
    return &map_metadata_;
}

double temperature_gradient_navigation::max_diff()
{
    static double max_diff, min_diff;
    auto diff_mat = temperature_map_ - temperature_map_prev_;
    cv::minMaxIdx(diff_mat, &min_diff, &max_diff);
    return max_diff;
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
    double distance = cv::norm(q1 - q2) * map_metadata_.resolution;
    return distance;
}

double temperature_gradient_navigation::calc_distance_sqrd(cv::Vec2i q1, cv::Vec2i q2)
{
    double tmp = calc_distance(q1, q2);
    return tmp * tmp;
}

double temperature_gradient_navigation::get_temperature(cv::Vec2i q)
{
    return temperature_map_.at<double>(q[1], q[0]);
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

const cv::Mat *temperature_gradient_navigation::get_map_ptr()
{
    return &map_;
}

const cv::Mat *temperature_gradient_navigation::get_temperaturemap_ptr()
{
    return &temperature_map_;
}

const cv::Mat *temperature_gradient_navigation::get_anglemap_ptr()
{
    return &anglemap_;
}

void temperature_gradient_navigation::clip(double &n, double lower, double upper)
{
    n = std::max(lower, std::min(n, upper));
}