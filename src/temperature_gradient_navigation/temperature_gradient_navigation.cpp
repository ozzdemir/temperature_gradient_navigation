#include "temperature_gradient_navigation/temperature_gradient_navigation.h"
#include "tf/tf.h"
temperature_gradient_navigation_::temperature_gradient_navigation_(ros::NodeHandle &nh, double hot_temperature, double cold_temperature, bool use_offline_map, bool visualization)
{
    nh_ = nh;
    hot_temperature_ = hot_temperature;
    cold_temperature_ = cold_temperature;
    visualization_ = visualization;
    temperature_map_initialized_ = goal_initialized_ = start_initialized_ = algorithm_initialized_ = no_solution_ = false;
    use_offline_map_ = use_offline_map;

    controller_timer_ = nh_.createTimer(ros::Duration(0.01), &temperature_gradient_navigation_::controller_cb, this);
    gradient_updating_timer_ = nh_.createTimer(ros::Duration(0.1), &temperature_gradient_navigation_::update_gradient, this);

    odom_subs_ = nh_.subscribe("/odom", 1, &temperature_gradient_navigation_::odom_cb, this);
    goalpose_subs_ = nh_.subscribe("/move_base_simple/goal", 1, &temperature_gradient_navigation_::goalpose_cb, this);
    initialpose_subs_ = nh_.subscribe("/initialpose", 1, &temperature_gradient_navigation_::initialpose_cb, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    gz_sms_cli_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    poll_trajectory_srv_ = nh_.advertiseService("/poll_trajectory", &temperature_gradient_navigation_::poll_trajectory_cb, this);
    get_trajectory_srv_ = nh_.advertiseService("/get_trajectory", &temperature_gradient_navigation_::get_trajectory_cb, this);

    if (use_offline_map)
    {
        static nav_msgs::GetMap payload;

        mapserver_cli_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");

        if (mapserver_cli_.waitForExistence(ros::Duration(5)))
        {
            mapserver_cli_.call(payload);
            map_cb(payload.response.map);
        }
        else
        {
            ROS_ERROR("No map found in map server. Exiting.");
        }
    }
    else
    {
        map_subs_ = nh_.subscribe("/map", 1, &temperature_gradient_navigation_::map_cb, this);
    }
}

void temperature_gradient_navigation_::odom_cb(const nav_msgs::Odometry &msg)
{
    static cv::Mat_<double> pos(3, 1);
    static cv::Mat_<double> pixel_pos(3, 1);
    position_ = msg.pose.pose.position;
    if (temperature_map_initialized_)
    {
        pos(0) = position_.x;
        pos(1) = position_.y;
        pos(2) = 1.0;
        pixel_pos = (real2pixel_tf_mat_ * pos);
        start_position_(0) = pixel_pos(0);
        start_position_(1) = pixel_pos(1);
    }
}

void temperature_gradient_navigation_::map_cb(const nav_msgs::OccupancyGrid &msg)
{
    int size_x = msg.info.width;
    int size_y = msg.info.height;
    N_ = size_x * size_y;
    cv::Mat map = cv::Mat(size_y, size_x, CV_8S, (void *)msg.data.data());
    map.setTo(25, (map == -1));
    map.convertTo(map, CV_8U);
    map.setTo(255, (map > 50));
    map.copyTo(map_);
    if (!temperature_map_initialized_)
    {
        map_metadata_ = msg.info;
        //std::cout << map_ << std::endl;
        temperature_map_initialized_ = true;
        temperature_map_ = cv::Mat(size_y, size_x, CV_64F, hot_temperature_);
        //temperature_map_.setTo(0, map > 50);
        anglemap_ = cv::Mat(size_y, size_x, CV_64F, 1e-36);
        magnitudemap_ = cv::Mat(size_y, size_x, CV_64F, 1e-36);
    }
    set_tf_mats(map_metadata_); // Set coordinate transform matrices
}

void temperature_gradient_navigation_::controller_cb(const ros::TimerEvent &evt)
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
    else
    {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_pub_.publish(cmd_vel_msg);
    }
}

void temperature_gradient_navigation_::initialpose_cb(const geometry_msgs::PoseWithCovarianceStamped &msg)
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
    std::cout << "Start: " << start_position_ << std::endl;
    // If offline map, check existence of solution via floodfill
    if (use_offline_map_)
    {
        cv::Mat temporary_map;
        map_.copyTo(temporary_map);
        cv::floodFill(temporary_map, {goal_position_(0), goal_position_(1)}, 77); // 77 is magic number
        if (temporary_map.at<char>(start_position_(1), start_position_(0)) != 77)
        {
            ROS_ERROR("No Solution exists from given starting point!");
            no_solution_ = true;
        }
        else
        {
            no_solution_ = false;
            traverse_ideal(start_position_);
        }
    }
}

void temperature_gradient_navigation_::goalpose_cb(const geometry_msgs::PoseStamped &msg)
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
    count_ = 0;
    std::cout << "Goal: " << goal_position_ << std::endl;
    temperature_map_.at<double>(goal_position_(1), goal_position_(0)) = cold_temperature_;
    // If offline map, check existence of solution via floodfill
    if (use_offline_map_)
    {
        cv::Mat temporary_map;
        map_.copyTo(temporary_map);
        cv::floodFill(temporary_map, {goal_position_(0), goal_position_(1)}, 77); // 77 is magic number
        if (temporary_map.at<char>(start_position_(1), start_position_(0)) != 77)
        {
            ROS_ERROR("No Solution exists to given goal!");
            no_solution_ = true;
        }
        else
        {
            no_solution_ = false;
        }
    }
}

bool temperature_gradient_navigation_::poll_trajectory_cb(temperature_gradient_navigation::poll_trajectory::Request &req, temperature_gradient_navigation::poll_trajectory::Response &res)
{
    goal_position_(0) = req.qgoal[0];
    goal_position_(1) = req.qgoal[1];
    start_position_(0) = req.qstart[0];
    start_position_(1) = req.qstart[1];
    if (req.reset_temperatures)
    {
        temperature_map_.setTo(hot_temperature_);
        algorithm_initialized_ = false;
        count_ = 0;
        std::cout << "qgoal: " << goal_position_;
        std::cout << " qstart: [" << req.qstart[0] << ", " << req.qstart[1] << "]" << std::endl;
        // Check solution existence
        cv::Mat temporary_map;
        map_.copyTo(temporary_map);
        cv::floodFill(temporary_map, {goal_position_(0), goal_position_(1)}, 77); // 77 is magic number
        if (temporary_map.at<char>(start_position_(1), start_position_(0)) != 77)
        {
            ROS_ERROR("No Solution exists to given goal!");
            no_solution_ = true;
            res.solution_state = temperature_gradient_navigation::poll_trajectory::Response::NO_SOLUTION;
            return true;
        }
        else
        {
            no_solution_ = false;
        }
    }

    if (temperature_map_initialized_)
    {
        temperature_map_.at<double>(goal_position_(1), goal_position_(0)) = cold_temperature_;
        start_initialized_ = true;
        goal_initialized_ = true;
        double start_temperature = get_temperature(start_position_);

        std::cout << "start temperature: " << start_temperature << std::endl;
        if (start_temperature < hot_temperature_)
        {
            res.solution_state = temperature_gradient_navigation::poll_trajectory::Response::READY;
        }
        else
        {
            res.solution_state = temperature_gradient_navigation::poll_trajectory::Response::NOT_READY;
        }
    }
    else
    {
        res.solution_state = temperature_gradient_navigation::poll_trajectory::Response::NOT_READY;
    }
    return true;
}
bool temperature_gradient_navigation_::get_trajectory_cb(temperature_gradient_navigation::get_trajectory::Request &req, temperature_gradient_navigation::get_trajectory::Response &res)
{
    auto trajectory = traverse_ideal(start_position_);
    for (auto i = trajectory.begin(); i != trajectory.end(); ++i)
    {
        res.trajectory_x.push_back((*i)(0));
        res.trajectory_y.push_back((*i)(1));
    }

    return true;
}

void temperature_gradient_navigation_::set_tf_mats(nav_msgs::MapMetaData metadata)
{ // TODO: set matrices such that it is universal?
    double pixel2real_mat_elements[9] = {map_metadata_.resolution, 0, map_metadata_.origin.position.x,
                                         0, map_metadata_.resolution, map_metadata_.origin.position.y,
                                         0, 0, 1};
    double real2pixel_mat_elements[9] = {1 / map_metadata_.resolution, 0, -map_metadata_.origin.position.x / map_metadata_.resolution,
                                         0, 1 / map_metadata_.resolution, -map_metadata_.origin.position.y / map_metadata_.resolution,
                                         0, 0, 1};
    pixel2real_tf_mat_ = cv::Mat(3, 3, CV_64F, pixel2real_mat_elements).clone();
    real2pixel_tf_mat_ = cv::Mat(3, 3, CV_64F, real2pixel_mat_elements).clone();
}

void temperature_gradient_navigation_::update_gradient(const ros::TimerEvent &evt)
{
    if (algorithm_initialized_)
    {
        cv::Mat dx, dy;
        cv::Sobel(temperature_map_, dx, CV_64F, 1, 0, 1);
        cv::Sobel(temperature_map_, dy, CV_64F, 0, 1, 1);
        cv::cartToPolar(dx, dy, magnitudemap_, anglemap_); // Anglemap later can be used in traversal
    }
}

void temperature_gradient_navigation_::update_temperatures()
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

int temperature_gradient_navigation_::iterate_algorithm()
{
    static double epsilon = 1e-3;
    if (goal_initialized_)
    {
        if (!algorithm_initialized_)
        {
            count_++;
            update_temperatures();
            if (use_offline_map_ & no_solution_)
            {
                return state_no_path;
            }
            if (get_temperature(start_position_) == hot_temperature_)
            {
                if (count_ >= N_)
                {
                    return state_no_path;
                }
                return state_initializing;
            }
            else
            {
                //get_temperature(start_position_) == hot_temperature_;
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
        return state_waiting_goal;
    }
}

std::vector<cv::Vec2d> temperature_gradient_navigation_::traverse_ideal(cv::Vec2i qstart)
{
    static ros::Publisher trajectory_marker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization/trajectory_ideal", 1, true);
    std::vector<cv::Vec2d> trajectory;
    cv::Vec2d cur_pos;
    cur_pos(0) = qstart(0);
    cur_pos(1) = qstart(1);
    cv::Vec2i cur_pt = qstart;
    cv::Vec2i old_pt = cur_pt;
    double sinx, cosx;

    if (algorithm_initialized_)
    {
        try
        {
            for (int i = 0; i < 5000; i++)
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
                    break;
                }
            }
        }
        catch (std::exception &e)
        {
            std::cout << e.what() << std::endl;
        }
        visualization_msgs::Marker marker;
        geometry_msgs::Point tmp_pt;
        static cv::Mat_<double> pos(3, 1);
        static cv::Mat_<double> pixel_pos(3, 1);
        if (!no_solution_)
        {
            // Publish state marker
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
            }
            trajectory_marker_pub.publish(marker);
        }
        return trajectory;
    }
}

const nav_msgs::MapMetaData *temperature_gradient_navigation_::get_map_metadata()
{
    return &map_metadata_;
}

double temperature_gradient_navigation_::max_diff()
{
    static double max_diff, min_diff;
    static bool first_call = true;
    auto diff_mat = temperature_map_ - temperature_map_prev_;
    diff_mat = cv::abs(diff_mat) / temperature_map_;
    cv::minMaxIdx(diff_mat, &min_diff, &max_diff);
    std::cout << "maxdif: " << max_diff << std::endl;
    std::cout << "mindiff: " << min_diff << std::endl;
    return max_diff;
}

double temperature_gradient_navigation_::get_gradient_angle(cv::Vec2i q)
{
    return anglemap_.at<double>(q[1], q[0]);
}

double temperature_gradient_navigation_::get_gradient_magnitude(cv::Vec2i q)
{
    return magnitudemap_.at<double>(q[1], q[0]);
}

double temperature_gradient_navigation_::calc_distance(cv::Vec2i q1, cv::Vec2i q2)
{
    double distance = cv::norm(q1 - q2) * map_metadata_.resolution;
    return distance;
}

double temperature_gradient_navigation_::calc_distance_sqrd(cv::Vec2i q1, cv::Vec2i q2)
{
    double tmp = calc_distance(q1, q2);
    return tmp * tmp;
}

double temperature_gradient_navigation_::get_temperature(cv::Vec2i q)
{
    return temperature_map_.at<double>(q[1], q[0]);
}

bool temperature_gradient_navigation_::gz_set_position(double x, double y)
{
    static gazebo_msgs::SetModelState sms_msg;
    sms_msg.request.model_state.model_name = "agent";
    sms_msg.request.model_state.pose.position.x = x;
    sms_msg.request.model_state.pose.position.y = y;
    sms_msg.request.model_state.reference_frame = "map";
    bool ret = gz_sms_cli_.call(sms_msg);
    return ret;
}

const cv::Mat *temperature_gradient_navigation_::get_map_ptr()
{
    return &map_;
}

const cv::Mat *temperature_gradient_navigation_::get_temperaturemap_ptr()
{
    return &temperature_map_;
}

const cv::Mat *temperature_gradient_navigation_::get_anglemap_ptr()
{
    return &anglemap_;
}

void temperature_gradient_navigation_::clip(double &n, double lower, double upper)
{
    n = std::max(lower, std::min(n, upper));
}
