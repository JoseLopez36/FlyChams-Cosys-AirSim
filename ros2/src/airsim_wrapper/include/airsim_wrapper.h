#pragma once

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

// Standard includes
#include <memory>
#include <mutex>

// ROS2 core headers
#include "rclcpp/rclcpp.hpp"

// ROS2 message headers
#include <rosgraph_msgs/msg/clock.hpp>

// TF2 headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// AirSim headers
#include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "math_common.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

// Global simulation control
#include <airsim_interfaces/srv/reset.hpp>
#include <airsim_interfaces/srv/run.hpp>
#include <airsim_interfaces/srv/pause.hpp>

// Vehicle camera control
#include <airsim_interfaces/srv/camera_capture.hpp>
#include <airsim_interfaces/msg/gimbal_angle_cmd.hpp>
#include <airsim_interfaces/msg/camera_fov_cmd.hpp>

// Target/cluster management
#include <airsim_interfaces/srv/add_target_group.hpp>
#include <airsim_interfaces/srv/add_cluster_group.hpp>
#include <airsim_interfaces/srv/remove_all_targets.hpp>
#include <airsim_interfaces/srv/remove_all_clusters.hpp>
#include <airsim_interfaces/msg/update_target_cmd_group.hpp>
#include <airsim_interfaces/msg/update_cluster_cmd_group.hpp>

namespace airsim_wrapper
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief AirSim ROS2 Wrapper Class
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-31
     * ════════════════════════════════════════════════════════════════
     */
    class AirsimWrapper
    {
    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Type Aliases
    public:
        using AirSimSettings = msr::airlib::AirSimSettings;

    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Constructors/Destructors
    public:
        AirsimWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip, uint16_t host_port);
        ~AirsimWrapper();
        void shutdown();

    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Initialization methods
    public:
        void initialize_airsim();
        void initialize_ros();

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: ROS Callbacks
    private:
        /// Timer callbacks
        void clock_timer_cb();

        /// Subscriber callbacks
        void gimbal_angle_cmd_cb(const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr gimbal_angle_cmd_msg);
        void camera_fov_cmd_cb(const airsim_interfaces::msg::CameraFovCmd::SharedPtr camera_fov_cmd_msg);
        void update_target_cmd_group_cb(const airsim_interfaces::msg::UpdateTargetCmdGroup::SharedPtr update_target_cmd_group_msg);
        void update_cluster_cmd_group_cb(const airsim_interfaces::msg::UpdateClusterCmdGroup::SharedPtr update_cluster_cmd_group_msg);

        /// Service callbacks
        bool reset_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request, const std::shared_ptr<airsim_interfaces::srv::Reset::Response> response);
        bool run_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Run::Request> request, const std::shared_ptr<airsim_interfaces::srv::Run::Response> response);
        bool pause_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Pause::Request> request, const std::shared_ptr<airsim_interfaces::srv::Pause::Response> response);
        bool camera_capture_srv_cb(const std::shared_ptr<airsim_interfaces::srv::CameraCapture::Request> request, const std::shared_ptr<airsim_interfaces::srv::CameraCapture::Response> response);
        bool add_target_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Response> response);
        bool add_cluster_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Response> response);
        bool remove_all_targets_cb(const std::shared_ptr<airsim_interfaces::srv::RemoveAllTargets::Request> request, const std::shared_ptr<airsim_interfaces::srv::RemoveAllTargets::Response> response);
        bool remove_all_clusters_cb(const std::shared_ptr<airsim_interfaces::srv::RemoveAllClusters::Request> request, const std::shared_ptr<airsim_interfaces::srv::RemoveAllClusters::Response> response);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: AirSim Client Methods
    private:
        rclcpp::Time client_get_timestamp();
        void client_reset();
        void client_pause(const bool& is_paused);
        void client_set_gimbal_attitude(const msr::airlib::Quaternionr& attitude, const std::string& camera_name, const std::string& vehicle_name);
        void client_set_camera_fov(const std::string& camera_name, const float& fov, const std::string& vehicle_name);
        bool client_set_agent_cameras_active(const std::string& vehicle_name, const bool& active);
        void client_add_targets(const std::vector<std::string>& target_names, const std::vector<std::string>& target_types, const std::vector<msr::airlib::Vector3r>& positions, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba);
        void client_add_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba);
        void client_remove_all_targets();
        void client_remove_all_clusters();
        void client_update_targets(const std::vector<std::string>& target_names, const std::vector<msr::airlib::Vector3r>& positions);
        void client_update_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Utility Methods
    private:
        rclcpp::Time get_sim_clock_time();
        msr::airlib::Vector3r get_airlib_point(const geometry_msgs::msg::Point& geometry_msgs_point) const;
        std::vector<msr::airlib::Vector3r> get_airlib_points(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const;
        std::vector<float> get_airlib_color(const std_msgs::msg::ColorRGBA& std_msgs_color) const;
        std::vector<std::vector<float>> get_airlib_colors(const std::vector<std_msgs::msg::ColorRGBA>& std_msgs_colors) const;
        msr::airlib::Quaternionr get_gimbal_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: ROS Components
    private:
        // Clock publisher
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

        // Global simulation services
        rclcpp::Service<airsim_interfaces::srv::Reset>::SharedPtr reset_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Run>::SharedPtr run_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Pause>::SharedPtr pause_srvr_;

        // Vehicle camera service
        rclcpp::Service<airsim_interfaces::srv::CameraCapture>::SharedPtr camera_capture_srvr_;

        // Vehicle camera subscribers
        rclcpp::Subscription<airsim_interfaces::msg::GimbalAngleCmd>::SharedPtr gimbal_angle_cmd_sub_;
        rclcpp::Subscription<airsim_interfaces::msg::CameraFovCmd>::SharedPtr camera_fov_cmd_sub_;

        // Tracking services
        rclcpp::Service<airsim_interfaces::srv::AddTargetGroup>::SharedPtr add_target_group_srvr_;
        rclcpp::Service<airsim_interfaces::srv::AddClusterGroup>::SharedPtr add_cluster_group_srvr_;
        rclcpp::Service<airsim_interfaces::srv::RemoveAllTargets>::SharedPtr remove_all_targets_srvr_;
        rclcpp::Service<airsim_interfaces::srv::RemoveAllClusters>::SharedPtr remove_all_clusters_srvr_;

        // Tracking subscribers
        rclcpp::Subscription<airsim_interfaces::msg::UpdateTargetCmdGroup>::SharedPtr update_target_cmd_group_sub_;
        rclcpp::Subscription<airsim_interfaces::msg::UpdateClusterCmdGroup>::SharedPtr update_cluster_cmd_group_sub_;

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Member Variables
    private:
        // Settings and configuration
        AirSimSettingsParser airsim_settings_parser_;
        std::string host_ip_;
        uint16_t host_port_;
        double update_sim_clock_every_n_sec_;

        // AirSim clients
        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_control_;
        std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_tracking_;
        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_clock_;

        // Node handle
        std::shared_ptr<rclcpp::Node> nh_;

        // Timers
        rclcpp::TimerBase::SharedPtr sim_clock_update_timer_;

        // Message storage
        rosgraph_msgs::msg::Clock ros_clock_;
        std::mutex clock_mutex_;
    };

} // namespace airsim_wrapper