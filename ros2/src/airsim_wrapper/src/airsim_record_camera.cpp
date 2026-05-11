#include "rclcpp/rclcpp.hpp"

// Standard headers
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

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

class AirSimCameraRecorder : public rclcpp::Node
{
public:
    AirSimCameraRecorder() : Node("airsim_camera_recorder")
    {
        // Declare parameters with defaults
        this->declare_parameter("host_ip", "localhost");
        this->declare_parameter("host_port", 41451);
        this->declare_parameter("camera_name", "");
        this->declare_parameter("images_root", "./images");
        this->declare_parameter("fps", 10.0);

        // Get parameters
        host_ip_ = this->get_parameter("host_ip").as_string();
        host_port_ = this->get_parameter("host_port").as_int();
        camera_name_ = this->get_parameter("camera_name").as_string();
        images_root_ = this->get_parameter("images_root").as_string();
        fps_ = this->get_parameter("fps").as_double();

        // Create images directory if it doesn't exist
        std::filesystem::create_directories(images_root_ + "/" + camera_name_);

        // Create and connect to airsim client
        try {
            airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
            airsim_client_->confirmConnection();
            RCLCPP_INFO(this->get_logger(), "Connected to AirSim at %s:%d", host_ip_.c_str(), host_port_);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to AirSim: %s", e.what());
            return;
        }

        // Calculate timer period from fps
        auto timer_period = std::chrono::duration<double>(1.0 / fps_);

        // Create timer for image capture
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
            std::bind(&AirSimCameraRecorder::capture_image, this));

        RCLCPP_INFO(this->get_logger(), "Starting camera recording at %.2f fps", fps_);
        RCLCPP_INFO(this->get_logger(), "Camera: %s", camera_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Images directory: %s", images_root_.c_str());
    }

private:
    void capture_image()
    {
        try {
            // Get current timestamp
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;

            // Format timestamp for filename
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
            ss << "_" << std::setfill('0') << std::setw(3) << ms.count();

            std::string filename = "image_" + ss.str() + ".png";
            std::string full_path = images_root_ + "/" + camera_name_ + "/" + filename;

            // Get image from AirSim
            std::vector<uint8_t> image = airsim_client_->simGetImage(
                camera_name_, msr::airlib::ImageCaptureBase::ImageType::Scene);

            if (image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty image from camera %s", camera_name_.c_str());
                return;
            }

            // Save image
            std::ofstream image_file(full_path, std::ios::binary);
            image_file.write(reinterpret_cast<const char*>(image.data()), image.size());
            image_file.close();

            image_count_++;
            RCLCPP_INFO(this->get_logger(), "Saved image #%d: %s", image_count_, filename.c_str());

        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error capturing image: %s", e.what());
        }
    }

    // Member variables
    std::string host_ip_;
    int host_port_;
    std::string camera_name_;
    std::string images_root_;
    double fps_;
    int image_count_ = 0;

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create and run the node
    auto node = std::make_shared<AirSimCameraRecorder>();

    // Spin the node
    rclcpp::spin(node);

    // Cleanup
    rclcpp::shutdown();
    return 0;
}