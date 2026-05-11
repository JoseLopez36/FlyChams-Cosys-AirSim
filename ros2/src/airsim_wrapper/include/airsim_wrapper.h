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
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <atomic>

// ROS2 core headers
#include "rclcpp/rclcpp.hpp"

// ROS2 message headers
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

// TF2 headers
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// AirSim headers
#include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "math_common.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

// Third party headers
#include "yaml-cpp/yaml.h"

// Global commands
#include <airsim_interfaces/srv/load_level.hpp>
#include <airsim_interfaces/srv/reset.hpp>
#include <airsim_interfaces/srv/run.hpp>
#include <airsim_interfaces/srv/pause.hpp>

// Vehicle commands
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>
#include <airsim_interfaces/srv/hover.hpp>
#include <airsim_interfaces/srv/arm_disarm.hpp>
#include <airsim_interfaces/srv/enable_control.hpp>
#include <airsim_interfaces/srv/camera_capture.hpp>
#include <airsim_interfaces/msg/vel_cmd.hpp>
#include <airsim_interfaces/msg/pos_cmd.hpp>
#include <airsim_interfaces/msg/gimbal_angle_cmd.hpp>
#include <airsim_interfaces/msg/camera_fov_cmd.hpp>
#include <airsim_interfaces/msg/camera_orientation.hpp>

// Window commands
#include <airsim_interfaces/msg/window_image_cmd_group.hpp>
#include <airsim_interfaces/msg/window_rectangle_cmd.hpp>
#include <airsim_interfaces/msg/window_string_cmd.hpp>

// Tracking commands
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
        using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;
        using CameraSetting = msr::airlib::AirSimSettings::CameraSetting;
        using CaptureSetting = msr::airlib::AirSimSettings::CaptureSetting;

    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Constructors/Destructors
    public:
        AirsimWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip, uint16_t host_port, bool broadcast_transforms);
        ~AirsimWrapper();
        void shutdown();

    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Initialization methods
    public:
        void initialize_airsim();
        void initialize_ros();

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Vehicle Classes
    private:
        /**
         * Class for camera ROS interfaces
         */
        class CameraROS
        {
        public:
            virtual ~CameraROS() = default;
            std::string camera_name;

            // Camera setting
            CameraSetting camera_setting;
            // Transforms
            geometry_msgs::msg::TransformStamped body_tf_msg;
            geometry_msgs::msg::TransformStamped optical_static_tf_msg;
            // Frame IDs
            std::string body_frame_id;
            std::string optical_frame_id;
        };

        /**
         * Class for vehicle ROS interfaces
         */
        class VehicleROS
        {
        public:
            virtual ~VehicleROS() = default;
            std::string vehicle_name;

            // Vehicle data
            nav_msgs::msg::Odometry global_odom;
            nav_msgs::msg::Odometry local_odom;
            airsim_interfaces::msg::CameraOrientation camera_orientation;
            // Vehicle setting
            VehicleSetting vehicle_setting;
            // Camera data
            std::unordered_map<std::string, std::unique_ptr<CameraROS>> camera_map;
            // Publisher
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom_pub;
            rclcpp::Publisher<airsim_interfaces::msg::CameraOrientation>::SharedPtr camera_orientation_pub;
            // Subscriber
            rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr local_vel_cmd_sub;
            rclcpp::Subscription<airsim_interfaces::msg::PosCmd>::SharedPtr local_pos_cmd_sub;
            rclcpp::Subscription<airsim_interfaces::msg::PosCmd>::SharedPtr global_pos_cmd_sub;
            rclcpp::Subscription<airsim_interfaces::msg::GimbalAngleCmd>::SharedPtr gimbal_angle_cmd_sub;
            rclcpp::Subscription<airsim_interfaces::msg::CameraFovCmd>::SharedPtr camera_fov_cmd_sub;
            // Transforms
            geometry_msgs::msg::TransformStamped local_static_tf_msg;
            geometry_msgs::msg::TransformStamped body_tf_msg;
            // Frame IDs
            std::string local_frame_id;
            std::string body_frame_id;
        };

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: ROS Callbacks
    private:
        /// Timer callbacks
        void state_timer_cb();
        void clock_timer_cb();

        /// Subscriber callbacks
        void local_vel_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::VelCmd::SharedPtr vel_cmd_msg);
        void local_pos_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::PosCmd::SharedPtr pos_cmd_msg);
        void global_pos_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::PosCmd::SharedPtr pos_cmd_msg);
        void gimbal_angle_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr gimbal_angle_cmd_msg);
        void camera_fov_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::CameraFovCmd::SharedPtr camera_fov_cmd_msg);
        void window_image_cmd_group_cb(const airsim_interfaces::msg::WindowImageCmdGroup::SharedPtr window_image_cmd_group_msg);
        void window_rectangle_cmd_cb(const airsim_interfaces::msg::WindowRectangleCmd::SharedPtr window_rectangle_cmd_msg);
        void window_string_cmd_cb(const airsim_interfaces::msg::WindowStringCmd::SharedPtr window_string_cmd_msg);
        void update_target_cmd_group_cb(const airsim_interfaces::msg::UpdateTargetCmdGroup::SharedPtr update_target_cmd_group_msg);
        void update_cluster_cmd_group_cb(const airsim_interfaces::msg::UpdateClusterCmdGroup::SharedPtr update_cluster_cmd_group_msg);

        /// Service callbacks
        bool load_level_srv_cb(const std::shared_ptr<airsim_interfaces::srv::LoadLevel::Request> request, const std::shared_ptr<airsim_interfaces::srv::LoadLevel::Response> response);
        bool reset_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request, const std::shared_ptr<airsim_interfaces::srv::Reset::Response> response);
        bool run_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Run::Request> request, const std::shared_ptr<airsim_interfaces::srv::Run::Response> response);
        bool pause_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Pause::Request> request, const std::shared_ptr<airsim_interfaces::srv::Pause::Response> response);
        bool enable_control_srv_cb(const std::shared_ptr<airsim_interfaces::srv::EnableControl::Request> request, const std::shared_ptr<airsim_interfaces::srv::EnableControl::Response> response);
        bool arm_disarm_srv_cb(const std::shared_ptr<airsim_interfaces::srv::ArmDisarm::Request> request, const std::shared_ptr<airsim_interfaces::srv::ArmDisarm::Response> response);
        bool takeoff_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request, const std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);
        bool land_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Land::Request> request, const std::shared_ptr<airsim_interfaces::srv::Land::Response> response);
        bool hover_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Hover::Request> request, const std::shared_ptr<airsim_interfaces::srv::Hover::Response> response);
        bool camera_capture_srv_cb(const std::shared_ptr<airsim_interfaces::srv::CameraCapture::Request> request, const std::shared_ptr<airsim_interfaces::srv::CameraCapture::Response> response);
        bool add_target_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Response> response);
        bool add_cluster_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Response> response);
        bool remove_all_targets_cb(const std::shared_ptr<airsim_interfaces::srv::RemoveAllTargets::Request> request, const std::shared_ptr<airsim_interfaces::srv::RemoveAllTargets::Response> response);
        bool remove_all_clusters_cb(const std::shared_ptr<airsim_interfaces::srv::RemoveAllClusters::Request> request, const std::shared_ptr<airsim_interfaces::srv::RemoveAllClusters::Response> response);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: State Update
    private:
        void update_vehicle_odom(VehicleROS* vehicle_ros, const msr::airlib::Kinematics::State& kinematics_estimated);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: State Publish
    private:
        void publish_vehicle_state();

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Settings and Configuration
    private:
        void create_ros_comms_from_settings_json();
        void initialize_vehicle_odom(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting);
        void initialize_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting);
        void initialize_camera_tf(VehicleROS* vehicle_ros, CameraROS* camera_ros, const CameraSetting& camera_setting);
        void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
        void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;
        geometry_msgs::msg::Transform get_camera_optical_tf() const;

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: AirSim Client Methods
    private:
        // State methods
        rclcpp::Time client_get_timestamp();
        bool client_get_paused();
        bool client_get_enabled_control(const std::string& vehicle_name);
        msr::airlib::MultirotorState client_get_multirotor_state(const std::string& vehicle_name);
        msr::airlib::MultirotorState client_get_ground_truth_multirotor_state(const std::string& vehicle_name);
        msr::airlib::Pose client_get_camera_pose(const std::string& vehicle_name, const std::string& camera_name);

        // Global methods
        void client_load_level(const std::string& level_name);
        void client_reset();
        void client_pause(const bool& is_paused);

        // Vehicle methods
        void client_enable_control(const bool& enable, const std::string& vehicle_name);
        void client_arm_disarm(const bool& arm, const std::string& vehicle_name);
        void client_takeoff(const float& timeout, const std::string& vehicle_name, const bool& wait = false);
        void client_land(const float& timeout, const std::string& vehicle_name, const bool& wait = false);
        void client_hover(const std::string& vehicle_name);
        void client_move_by_velocity(const float& vx, const float& vy, const float& vz, const float& dt, const std::string& vehicle_name);
        void client_move_by_position(const float& x, const float& y, const float& z, const float& vel, const float& timeout, const std::string& vehicle_name);
        void client_set_gimbal_attitude(const msr::airlib::Quaternionr& attitude, const std::string& camera_name, const std::string& vehicle_name);
        void client_set_camera_fov(const std::string& camera_name, const float& fov, const std::string& vehicle_name);
        bool client_set_agent_cameras_active(const std::string& vehicle_name, const bool& active);

        // Window methods
        void client_set_window_images(const std::vector<int>& window_indices, const std::vector<std::string>& vehicle_names, const std::vector<std::string>& camera_names, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes);
        void client_set_window_rectangles(const int& window_index, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes, const std::vector<float>& color, const float& thickness);
        void client_set_window_strings(const int& window_index, const std::vector<std::string>& strings, const std::vector<msr::airlib::Vector2r>& positions, const std::vector<float>& color, const float& scale);

        // Tracking methods
        void client_add_targets(const std::vector<std::string>& target_names, const std::vector<std::string>& target_types, const std::vector<msr::airlib::Vector3r>& positions, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba);
        void client_add_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba);
        void client_remove_targets(const std::vector<std::string>& target_names);
        void client_remove_clusters(const std::vector<std::string>& cluster_names);
        void client_remove_all_targets();
        void client_remove_all_clusters();
        void client_update_targets(const std::vector<std::string>& target_names, const std::vector<msr::airlib::Vector3r>& positions);
        void client_update_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Utility Methods
    private:
        rclcpp::Time get_sim_clock_time();
        msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;
        msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat) const;
        msr::airlib::Pose get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const;
        msr::airlib::Pose get_airlib_pose(const geometry_msgs::msg::Pose& geometry_msgs_pose) const;
        msr::airlib::Vector3r get_airlib_rpy(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;
        msr::airlib::Vector3r get_airlib_point(const geometry_msgs::msg::Point& geometry_msgs_point) const;
        msr::airlib::Vector2r get_airlib_point_2d(const geometry_msgs::msg::Point& geometry_msgs_point) const;
        std::vector<msr::airlib::Vector3r> get_airlib_points(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const;
        std::vector<msr::airlib::Vector2r> get_airlib_points_2d(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const;
        std::vector<float> get_airlib_color(const std_msgs::msg::ColorRGBA& std_msgs_color) const;
        std::vector<std::vector<float>> get_airlib_colors(const std::vector<std_msgs::msg::ColorRGBA>& std_msgs_colors) const;
        geometry_msgs::msg::Transform get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation);
        geometry_msgs::msg::Transform get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion);
        geometry_msgs::msg::Pose get_pose_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation);
        geometry_msgs::msg::Pose get_pose_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion);
        geometry_msgs::msg::Point transform_position_to_local(const geometry_msgs::msg::Point& global_point, const std::string& local_frame) const;
        geometry_msgs::msg::Pose transform_pose_to_global(const geometry_msgs::msg::Pose& local_pose, const std::string& local_frame) const;
        geometry_msgs::msg::Twist transform_twist_to_global(const geometry_msgs::msg::Twist& local_twist, const std::string& local_frame) const;
        geometry_msgs::msg::Transform get_gimbal_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion);
        msr::airlib::Quaternionr get_gimbal_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: ROS Components
    private:
        // Clock publisher
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

        // Global services
        rclcpp::Service<airsim_interfaces::srv::LoadLevel>::SharedPtr load_level_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Reset>::SharedPtr reset_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Run>::SharedPtr run_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Pause>::SharedPtr pause_srvr_;

        // Vehicle services
        rclcpp::Service<airsim_interfaces::srv::EnableControl>::SharedPtr enable_control_srvr_;
        rclcpp::Service<airsim_interfaces::srv::ArmDisarm>::SharedPtr arm_disarm_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Hover>::SharedPtr hover_srvr_;
        rclcpp::Service<airsim_interfaces::srv::CameraCapture>::SharedPtr camera_capture_srvr_;

        // Window subscribers
        rclcpp::Subscription<airsim_interfaces::msg::WindowImageCmdGroup>::SharedPtr window_image_cmd_group_sub_;
        rclcpp::Subscription<airsim_interfaces::msg::WindowRectangleCmd>::SharedPtr window_rectangle_cmd_sub_;
        rclcpp::Subscription<airsim_interfaces::msg::WindowStringCmd>::SharedPtr window_string_cmd_sub_;

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
        bool broadcast_transforms_;
        double update_airsim_state_every_n_sec_;
        double update_sim_clock_every_n_sec_;

        // Vehicle data
        std::unordered_map<std::string, std::unique_ptr<VehicleROS>> vehicle_map_;

        // AirSim clients
        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_state_;
        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_control_;
        std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_window_;
        std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_tracking_;
        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_clock_;

        // Node and callback groups
        std::shared_ptr<rclcpp::Node> nh_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_state_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_control_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_window_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_tracking_;

        // TF components
        std::string world_frame_id_ = "world";
        std::string vehicle_local_frame_id_ = "local";
        std::string vehicle_body_frame_id_ = "body";
        std::string camera_body_frame_id_ = "body";
        std::string camera_optical_frame_id_ = "optical";
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        // Timers
        rclcpp::TimerBase::SharedPtr airsim_state_update_timer_;
        rclcpp::TimerBase::SharedPtr sim_clock_update_timer_;
        rclcpp::TimerBase::SharedPtr airsim_status_update_timer_;
        rclcpp::TimerBase::SharedPtr world_plot_update_timer_;

        // Message storage
        rosgraph_msgs::msg::Clock ros_clock_;
        std::mutex clock_mutex_;
    };

} // namespace airsim_wrapper