#pragma once

#include "rclcpp/rclcpp.hpp"
#include "aeb_cpp/aeb.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

class AebNode : public rclcpp::Node {
public:
    explicit AebNode(const rclcpp::NodeOptions &options);

    void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr scan);

    void handle_odom(nav_msgs::msg::Odometry::SharedPtr odom);

private:
    /// Backing AEB object
    std::unique_ptr<Aeb<71>> _aeb;
    /// Wheelbase of the vehicle, in meters.
    double _wheelbase;
    /// Frame where range values are from.
    std::string _lidar_frame;
    /// Min allowed TTC
    double _min_ttc;
    /// Size of steps to take during prediction, in ms.
    size_t _step_size;

    std::unique_ptr<tf2_ros::Buffer> _tf_buff;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listen;

    /// Incoming laserscans
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _ls_sub;
    /// Incoming Odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    /// Sends true when we should stop
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr _estop_pub;
};
