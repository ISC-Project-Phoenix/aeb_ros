#include "aeb_ros/aeb_node.h"

#include <functional>
#include <tuple>

#include "aeb_ros/util.hpp"
#include "rclcpp/qos.hpp"

using namespace LineDrawing;

AebNode::AebNode(const rclcpp::NodeOptions& options) : rclcpp::Node("Aeb", options) {
    // Node setup
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Wheelbase of the vehicle";
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        this->_wheelbase = this->declare_parameter("wheelbase", 1.8, param_desc);
    }

    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Frame where range data comes from";
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        this->_lidar_frame = this->declare_parameter("lidar_frame", "laser_link", param_desc);
    }

    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Minimum allowed TTC (a TTC under this will trigger braking), in seconds";
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        this->_min_ttc = this->declare_parameter("min_ttc", 2.0, param_desc);
    }

    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description =
            "Size of a step in forward prediction, in ms. The higher this value, the lower the precision but the "
            "better the performance.";
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->_step_size = this->declare_parameter("step_size", 50, param_desc);
    }

    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Start of desired scan region, in degrees.";
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->_scan_range_start = this->declare_parameter("scan_region_start", 360 - 25, param_desc);
    }

    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "End of desired scan region, in degrees.";
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->_scan_range_end = this->declare_parameter("scan_region_end", 25, param_desc);
    }

    // Parse collision box
    auto param_desc_coll = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_coll.description =
        "Collision box of the vehicle, defined as a string of the top left and bottom right points wrt base_link. Ex "
        "[[1,2],[-2,3]]";
    param_desc_coll.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    auto coll_str =
        this->declare_parameter<std::string>("collision_box", "[[-0.675, 1.43],[0.675, -0.59]]", param_desc_coll);
    auto coll = parse_collision_box(coll_str);

    if (!coll.has_value()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to parse collision box!");
        std::terminate();
    }

    // Get axel to lidar transform
    this->_tf_buff = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->_tf_listen = std::make_unique<tf2_ros::TransformListener>(*this->_tf_buff);

    // Keep attempting transform until link appears (this avoids crashing while sim is loading)
    while (true) {
        try {
            _tf_buff->lookupTransform(this->_lidar_frame, "base_link", rclcpp::Time{0});
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Errored on frame transform, attempting again...");
            using namespace std::chrono_literals;
            // Avoid spamming output
            rclcpp::sleep_for(500ms);
            continue;
        }
        break;
    }

    auto basetlidar = _tf_buff->lookupTransform(this->_lidar_frame, "base_link", rclcpp::Time{0});

    this->_aeb = std::make_unique<Aeb<411>>(0, 0, this->_wheelbase, *coll,
                                           KartPoint{static_cast<float>(basetlidar.transform.translation.x),
                                                     static_cast<float>(-basetlidar.transform.translation.y)},
                                           this->_min_ttc, this->_step_size);

    this->_ls_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort(), std::bind(&AebNode::handle_scan, this, std::placeholders::_1));
    this->_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10).best_effort(), std::bind(&AebNode::handle_odom, this, std::placeholders::_1));
    this->_estop_pub = this->create_publisher<std_msgs::msg::UInt32>("/aeb_out", 10);
}

/// Returns true if the scan lies in our desired range.
bool AebNode::scan_in_range(double start, double end) const {
    // The only valid config where end < start is split over 360/0
    if (end < start) {
        // If either start or end lands in the range, then that means some part of the scan is in the region we want
        return (start >= this->_scan_range_start && start <= 360) || (start <= this->_scan_range_end && start >= 0) ||
               (end >= this->_scan_range_start && end <= 360) || (end <= this->_scan_range_end && end >= 0);
    } else {
        return (start >= this->_scan_range_start && start <= this->_scan_range_end) ||
               (end >= this->_scan_range_start && end <= this->_scan_range_end);
    }
}

/// Handles receiving lidarscans, spawning aeb if the desired region has been covered by the scanner.
void AebNode::handle_scan(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    static bool last_scan_in_bounds = false;
    static double last_start = 0.0;

    auto start = rad_to_deg(scan->angle_min);
    auto end = rad_to_deg(scan->angle_max);
    auto step = rad_to_deg(scan->angle_increment);

    // If any of our scans are in bounds, add those points to the next AEB run. This is to support LiDARs that give incremental
    // scans, rather than one large full scan. If scans have the same start, then that means we can just accept them all.
    if (this->scan_in_range(start, end) || last_start == start) {
        last_scan_in_bounds = true;

        double current_angle = start;
        std::vector<KartPoint> points{};

        // Convert scans into lib format
        for (float reading : scan->ranges) {
            // Filter out close points
            if (reading < 0.15) {
                continue;
            }

            auto polar_a = lidarscan_polar(current_angle);
            auto kartpoint = KartPoint::from_polar(reading, static_cast<float>(polar_a));
            points.push_back(kartpoint);

            current_angle += step;
        }

        this->_aeb->add_points(points.begin(), points.end());

        // If getting full scans, run AEB now to avoid starving the next else if
        if (last_start == start) {
            // I think this is actually cleaner
            goto aeb;
        }
    }
    // Run AEB if we have enough scans
    else if (last_scan_in_bounds) {
    aeb:
        last_scan_in_bounds = false;

        auto [collides, time] = this->_aeb->collision_check();

        // If we are going to collide, publish estop
        if (collides) {
            auto msg = std_msgs::msg::UInt32{};
            msg.data = static_cast<uint32_t>(time);
            this->_estop_pub->publish(msg);
        }
    }

    last_start = start;
}

void AebNode::handle_odom(nav_msgs::msg::Odometry::SharedPtr odom) {
    auto vel = odom->twist.twist.linear.x;
    auto steering = convert_trans_rot_vel_to_steering_angle(vel, odom->twist.twist.angular.z, this->_wheelbase);
    // Pass odom settings into AEB
    this->_aeb->update_steering(static_cast<float>(rad_to_deg(steering)));
    this->_aeb->update_velocity(static_cast<float>(vel));
}
