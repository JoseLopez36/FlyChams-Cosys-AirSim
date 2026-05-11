#include <rclcpp/rclcpp.hpp>

// Airsim wrapper include
#include "airsim_wrapper.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("airsim_node", node_options);
    std::string host_ip;
    uint16_t host_port = 41451;
    bool broadcast_transforms = false;
    nh->get_parameter("host_ip", host_ip);
    nh->get_parameter("host_port", host_port);
    nh->get_parameter("broadcast_transforms", broadcast_transforms);
    airsim_wrapper::AirsimWrapper airsim_wrapper(nh, host_ip, host_port, broadcast_transforms);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();

    return 0;
}