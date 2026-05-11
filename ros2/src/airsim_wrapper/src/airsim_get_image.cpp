#include "rclcpp/rclcpp.hpp"

// Standard headers
#include <fstream>

// AirSim headers
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "math_common.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create node
    auto node = rclcpp::Node::make_shared("airsim_get_image", options);

    // Get parameters - use get_parameter instead of declare_parameter to avoid redeclaration
    std::string host_ip;
    uint16_t host_port;
    std::string camera_name;
    std::string image_path;

    // Get parameters if they exist, otherwise use defaults
    if (!node->get_parameter("host_ip", host_ip)) {
        host_ip = "localhost";
    }
    if (!node->get_parameter("host_port", host_port)) {
        host_port = 41451;
    }
    if (!node->get_parameter("camera_name", camera_name)) {
        camera_name = "";
    }
    if (!node->get_parameter("image_path", image_path)) {
        image_path = "";
    }

    // Create and connect to airsim client
    auto airsim_client = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip, host_port));
    airsim_client->confirmConnection();

    // Get image
    std::vector<uint8_t> image = airsim_client->simGetImage(camera_name, msr::airlib::ImageCaptureBase::ImageType::Scene);

    // Save image
    std::ofstream image_file(image_path, std::ios::binary);
    image_file.write(reinterpret_cast<const char*>(image.data()), image.size());
    image_file.close();
    RCLCPP_INFO(node->get_logger(), "Image from camera %s saved to %s", camera_name.c_str(), image_path.c_str());

    // Finish the node
    RCLCPP_INFO(node->get_logger(), "Closing airsim_get_image_node");
    rclcpp::shutdown();
    return 0;
}