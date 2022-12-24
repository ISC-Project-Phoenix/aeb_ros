#include "aeb_ros/aeb_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto aeb = std::make_shared<AebNode>(options);
    exec.add_node(aeb);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
