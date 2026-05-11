#include <airsim_wrapper.h>

using namespace std::placeholders;

namespace airsim_wrapper
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR/DESTRUCTOR
    // ════════════════════════════════════════════════════════════════════════════

    AirsimWrapper::AirsimWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip, uint16_t host_port, bool broadcast_transforms)
        : airsim_settings_parser_(host_ip, host_port)
        , host_ip_(host_ip)
        , host_port_(host_port)
        , broadcast_transforms_(broadcast_transforms)
        , airsim_client_state_(nullptr)
        , airsim_client_control_(nullptr)
        , airsim_client_window_(nullptr)
        , airsim_client_tracking_(nullptr)
        , airsim_client_clock_(nullptr)
        , nh_(nh)
    {
        ros_clock_.clock = rclcpp::Time(0);

        if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeMultirotor)
        {
            RCLCPP_ERROR(nh_->get_logger(), "Unsupported simulation mode: %s", AirSimSettings::singleton().simmode_name.c_str());
            rclcpp::shutdown();
            return;
        }

        // Create TF broadcaster and static TF broadcaster
        if (broadcast_transforms_)
        {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);
        }
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize AirSim
        try
        {
            RCLCPP_INFO(nh_->get_logger(), "Initializing AirSim clients...");
            initialize_airsim();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            rclcpp::shutdown();
            return;
        }

        // Initialize ROS data and communications
        initialize_ros();

        // Create timers
        nh_->get_parameter("update_airsim_state_every_n_sec", update_airsim_state_every_n_sec_);
        airsim_state_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_airsim_state_every_n_sec_), std::bind(&AirsimWrapper::state_timer_cb, this), cb_state_);
        nh_->get_parameter("update_sim_clock_every_n_sec", update_sim_clock_every_n_sec_);
        sim_clock_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_sim_clock_every_n_sec_), std::bind(&AirsimWrapper::clock_timer_cb, this), cb_state_);
        RCLCPP_INFO(nh_->get_logger(), "Created state (%.4f) and clock (%.4f) timers", update_airsim_state_every_n_sec_, update_sim_clock_every_n_sec_);

        RCLCPP_INFO(nh_->get_logger(), "AirsimWrapper successfully initialized!");
    }

    AirsimWrapper::~AirsimWrapper()
    {
        // Shutdown AirSim
        shutdown();
    }

    void AirsimWrapper::shutdown()
    {
        // Destroy timers
        airsim_state_update_timer_.reset();
        sim_clock_update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::initialize_airsim()
    {
        // Create airsim clients
        airsim_client_state_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        airsim_client_control_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        airsim_client_window_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::RpcLibClientBase(host_ip_, host_port_));
        airsim_client_tracking_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::RpcLibClientBase(host_ip_, host_port_));
        airsim_client_clock_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));

        // Connect to AirSim in parallel
        std::vector<std::future<void>> futures;
        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_state_->confirmConnection();
            }));

        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_control_->confirmConnection();
            }));

        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_window_->confirmConnection();
            }));

        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_tracking_->confirmConnection();
            }));

        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_clock_->confirmConnection();
            }));

        // Wait for all clients to connect
        for (auto& future : futures)
        {
            future.get();
        }

        // Initialize sim clock
        ros_clock_.clock = client_get_timestamp();
    }

    void AirsimWrapper::initialize_ros()
    {
        // ROS params
        nh_->get_parameter_or("world_frame_id", world_frame_id_, world_frame_id_);
        nh_->get_parameter_or("vehicle_local_frame_id", vehicle_local_frame_id_, vehicle_local_frame_id_);
        nh_->get_parameter_or("vehicle_body_frame_id", vehicle_body_frame_id_, vehicle_body_frame_id_);
        nh_->get_parameter_or("camera_body_frame_id", camera_body_frame_id_, camera_body_frame_id_);
        nh_->get_parameter_or("camera_optical_frame_id", camera_optical_frame_id_, camera_optical_frame_id_);

        // Initialize callback groups
        cb_state_ = nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        cb_control_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_window_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_tracking_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create ROS communications
        create_ros_comms_from_settings_json();
    }

    void AirsimWrapper::create_ros_comms_from_settings_json()
    {
        // Get control subscriber options
        auto control_sub_options = rclcpp::SubscriptionOptions();
        control_sub_options.callback_group = cb_control_;

        // Get window subscriber options
        auto window_sub_options = rclcpp::SubscriptionOptions();
        window_sub_options.callback_group = cb_window_;

        // Get tracking subscriber options
        auto tracking_sub_options = rclcpp::SubscriptionOptions();
        tracking_sub_options.callback_group = cb_tracking_;

        // Initialize vehicle map from settings
        RCLCPP_INFO(nh_->get_logger(), "Initializing vehicle map from settings...");
        // Iterate over vehicles
        vehicle_map_.clear();
        for (const auto& [vehicle_name, vehicle_setting] : AirSimSettings::singleton().vehicles)
        {
            RCLCPP_INFO(nh_->get_logger(), "Initializing vehicle %s from settings...", vehicle_name.c_str());
            set_nans_to_zeros_in_pose(*vehicle_setting);

            // Initialize vehicle ROS instance
            std::unique_ptr<VehicleROS> vehicle_ros = std::unique_ptr<VehicleROS>(new VehicleROS());
            vehicle_ros->vehicle_name = vehicle_name;
            vehicle_ros->vehicle_setting = *vehicle_setting;
            vehicle_ros->local_frame_id = vehicle_name + "/" + vehicle_local_frame_id_;
            vehicle_ros->body_frame_id = vehicle_name + "/" + vehicle_body_frame_id_;
            initialize_vehicle_odom(vehicle_ros.get(), *vehicle_setting);
            initialize_vehicle_tf(vehicle_ros.get(), *vehicle_setting);

            // Initialize vehicle publishers
            vehicle_ros->global_odom_pub = nh_->create_publisher<nav_msgs::msg::Odometry>(vehicle_name + "/global/state/odom", 10);
            vehicle_ros->local_odom_pub = nh_->create_publisher<nav_msgs::msg::Odometry>(vehicle_name + "/local/state/odom", 10);
            vehicle_ros->camera_orientation_pub = nh_->create_publisher<airsim_interfaces::msg::CameraOrientation>(vehicle_name + "/cameras/state/orientation", 10);

            // Initialize vehicle subscribers
            vehicle_ros->local_vel_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::VelCmd>(
                vehicle_name + "/local/cmd/velocity", 10,
                [this, vehicle_name](const airsim_interfaces::msg::VelCmd::SharedPtr msg) {
                    this->local_vel_cmd_cb(vehicle_name, msg);
                },
                control_sub_options
            );
            vehicle_ros->local_pos_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::PosCmd>(
                vehicle_name + "/local/cmd/position", 10,
                [this, vehicle_name](const airsim_interfaces::msg::PosCmd::SharedPtr msg) {
                    this->local_pos_cmd_cb(vehicle_name, msg);
                },
                control_sub_options
            );
            vehicle_ros->global_pos_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::PosCmd>(
                vehicle_name + "/global/cmd/position", 10,
                [this, vehicle_name](const airsim_interfaces::msg::PosCmd::SharedPtr msg) {
                    this->global_pos_cmd_cb(vehicle_name, msg);
                },
                control_sub_options
            );
            vehicle_ros->gimbal_angle_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleCmd>(
                vehicle_name + "/gimbals/cmd/orientation", 10,
                [this, vehicle_name](const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr msg) {
                    this->gimbal_angle_cmd_cb(vehicle_name, msg);
                },
                control_sub_options
            );
            vehicle_ros->camera_fov_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::CameraFovCmd>(
                vehicle_name + "/cameras/cmd/fov", 10,
                [this, vehicle_name](const airsim_interfaces::msg::CameraFovCmd::SharedPtr msg) {
                    this->camera_fov_cmd_cb(vehicle_name, msg);
                },
                control_sub_options
            );

            // Iterate over camera map
            vehicle_ros->camera_map.clear();
            for (auto& [camera_name, camera_setting] : vehicle_setting->cameras)
            {
                if (!camera_setting.enable_gimbal)
                    continue;

                RCLCPP_INFO(nh_->get_logger(), "Initializing camera %s from settings...", camera_name.c_str());
                set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);

                // Initialize camera ROS instance
                auto camera_ros = std::unique_ptr<CameraROS>(new CameraROS());
                camera_ros->camera_name = camera_name;
                camera_ros->camera_setting = camera_setting;
                camera_ros->body_frame_id = vehicle_name + "/" + camera_name + "/" + camera_body_frame_id_;
                camera_ros->optical_frame_id = vehicle_name + "/" + camera_name + "/" + camera_optical_frame_id_;
                initialize_camera_tf(vehicle_ros.get(), camera_ros.get(), camera_setting);

                // Add camera to vehicle's camera map
                vehicle_ros->camera_map.emplace(camera_name, std::move(camera_ros));
            }

            // Add vehicle to map
            vehicle_map_.emplace(vehicle_name, std::move(vehicle_ros));
        }

        // Create ROS communications
        RCLCPP_INFO(nh_->get_logger(), "Creating ROS communications...");
        // Create clock publisher
        clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
        // Create global control services
        load_level_srvr_ = nh_->create_service<airsim_interfaces::srv::LoadLevel>("load_level", std::bind(&AirsimWrapper::load_level_srv_cb, this, _1, _2));
        reset_srvr_ = nh_->create_service<airsim_interfaces::srv::Reset>("reset", std::bind(&AirsimWrapper::reset_srv_cb, this, _1, _2));
        run_srvr_ = nh_->create_service<airsim_interfaces::srv::Run>("run", std::bind(&AirsimWrapper::run_srv_cb, this, _1, _2));
        pause_srvr_ = nh_->create_service<airsim_interfaces::srv::Pause>("pause", std::bind(&AirsimWrapper::pause_srv_cb, this, _1, _2));
        // Create vehicle services
        enable_control_srvr_ = nh_->create_service<airsim_interfaces::srv::EnableControl>("vehicles/cmd/enable_control", std::bind(&AirsimWrapper::enable_control_srv_cb, this, _1, _2));
        arm_disarm_srvr_ = nh_->create_service<airsim_interfaces::srv::ArmDisarm>("vehicles/cmd/arm_disarm", std::bind(&AirsimWrapper::arm_disarm_srv_cb, this, _1, _2));
        takeoff_srvr_ = nh_->create_service<airsim_interfaces::srv::Takeoff>("vehicles/cmd/takeoff", std::bind(&AirsimWrapper::takeoff_srv_cb, this, _1, _2));
        land_srvr_ = nh_->create_service<airsim_interfaces::srv::Land>("vehicles/cmd/land", std::bind(&AirsimWrapper::land_srv_cb, this, _1, _2));
        hover_srvr_ = nh_->create_service<airsim_interfaces::srv::Hover>("vehicles/cmd/hover", std::bind(&AirsimWrapper::hover_srv_cb, this, _1, _2));
        camera_capture_srvr_ = nh_->create_service<airsim_interfaces::srv::CameraCapture>("vehicles/cmd/camera_capture", std::bind(&AirsimWrapper::camera_capture_srv_cb, this, _1, _2));
        // Create window subscribers
        window_image_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::WindowImageCmdGroup>(
            "windows/cmd/image", 10, std::bind(&AirsimWrapper::window_image_cmd_group_cb, this, _1), window_sub_options);
        window_rectangle_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::WindowRectangleCmd>(
            "windows/cmd/rectangle", 10, std::bind(&AirsimWrapper::window_rectangle_cmd_cb, this, _1), window_sub_options);
        window_string_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::WindowStringCmd>(
            "windows/cmd/string", 10, std::bind(&AirsimWrapper::window_string_cmd_cb, this, _1), window_sub_options);
        // Create tracking services
        add_target_group_srvr_ = nh_->create_service<airsim_interfaces::srv::AddTargetGroup>("targets/cmd/add", std::bind(&AirsimWrapper::add_target_group_cb, this, _1, _2));
        add_cluster_group_srvr_ = nh_->create_service<airsim_interfaces::srv::AddClusterGroup>("clusters/cmd/add", std::bind(&AirsimWrapper::add_cluster_group_cb, this, _1, _2));
        remove_all_targets_srvr_ = nh_->create_service<airsim_interfaces::srv::RemoveAllTargets>("targets/cmd/remove_all", std::bind(&AirsimWrapper::remove_all_targets_cb, this, _1, _2));
        remove_all_clusters_srvr_ = nh_->create_service<airsim_interfaces::srv::RemoveAllClusters>("clusters/cmd/remove_all", std::bind(&AirsimWrapper::remove_all_clusters_cb, this, _1, _2));
        // Create tracking subscribers
        update_target_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::UpdateTargetCmdGroup>(
            "targets/cmd/update", 10, std::bind(&AirsimWrapper::update_target_cmd_group_cb, this, _1), tracking_sub_options);
        update_cluster_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::UpdateClusterCmdGroup>(
            "clusters/cmd/update", 10, std::bind(&AirsimWrapper::update_cluster_cmd_group_cb, this, _1), tracking_sub_options);
    }

    void AirsimWrapper::initialize_vehicle_odom(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
    {
        vehicle_ros->local_odom.header.frame_id = vehicle_ros->local_frame_id;
        vehicle_ros->local_odom.child_frame_id = vehicle_ros->body_frame_id;
        vehicle_ros->local_odom.header.stamp = get_sim_clock_time();
        vehicle_ros->local_odom.pose.pose = geometry_msgs::msg::Pose();
        vehicle_ros->local_odom.twist.twist = geometry_msgs::msg::Twist();

        vehicle_ros->global_odom.header.frame_id = world_frame_id_;
        vehicle_ros->global_odom.child_frame_id = vehicle_ros->body_frame_id;
        vehicle_ros->global_odom.header.stamp = get_sim_clock_time();
        vehicle_ros->global_odom.pose.pose = get_pose_msg_from_airsim(vehicle_setting.position, vehicle_setting.rotation);
        vehicle_ros->global_odom.twist.twist = geometry_msgs::msg::Twist();
    }

    void AirsimWrapper::initialize_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
    {
        auto& local_tf = vehicle_ros->local_static_tf_msg;
        auto& body_tf = vehicle_ros->body_tf_msg;

        // World to Local transform
        local_tf.header.frame_id = world_frame_id_;
        local_tf.child_frame_id = vehicle_ros->local_frame_id;
        local_tf.header.stamp = get_sim_clock_time();
        local_tf.transform = get_transform_msg_from_airsim(vehicle_setting.position, vehicle_setting.rotation);
        if (broadcast_transforms_)
        {
            static_tf_broadcaster_->sendTransform(local_tf);
        }

        // Local to Body transform
        body_tf.header.frame_id = vehicle_ros->local_frame_id;
        body_tf.child_frame_id = vehicle_ros->body_frame_id;
        body_tf.header.stamp = get_sim_clock_time();
        body_tf.transform = geometry_msgs::msg::Transform();
        if (broadcast_transforms_)
        {
            tf_broadcaster_->sendTransform(body_tf);
        }
    }

    void AirsimWrapper::initialize_camera_tf(VehicleROS* vehicle_ros, CameraROS* camera_ros, const CameraSetting& camera_setting)
    {
        auto& body_tf = camera_ros->body_tf_msg;
        auto& optical_tf = camera_ros->optical_static_tf_msg;

        // Vehicle Local to Camera Body transform
        body_tf.header.frame_id = vehicle_ros->local_frame_id;
        body_tf.child_frame_id = camera_ros->body_frame_id;
        body_tf.header.stamp = get_sim_clock_time();
        const auto& pose = client_get_camera_pose(vehicle_ros->vehicle_name, camera_ros->camera_name);
        body_tf.transform = get_gimbal_transform_msg_from_airsim(pose.position, pose.orientation);
        if (broadcast_transforms_)
        {
            tf_broadcaster_->sendTransform(body_tf);
        }

        // Body to Optical transform
        optical_tf.header.frame_id = camera_ros->body_frame_id;
        optical_tf.child_frame_id = camera_ros->optical_frame_id;
        optical_tf.header.stamp = get_sim_clock_time();
        optical_tf.transform = get_camera_optical_tf();
        if (broadcast_transforms_)
        {
            static_tf_broadcaster_->sendTransform(optical_tf);
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // COMMAND CALLBACKS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::local_vel_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::VelCmd::SharedPtr vel_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received local velocity command for vehicle %s", vehicle_name.c_str());

        if (!client_get_enabled_control(vehicle_name))
        {
            RCLCPP_ERROR(nh_->get_logger(), "Vehicle %s is not in OFFBOARD mode. Cannot send velocity command", vehicle_name.c_str());
            return;
        }

        // Extract message data
        const auto& vx = vel_cmd_msg->vel_cmd_x;
        const auto& vy = vel_cmd_msg->vel_cmd_y;
        const auto& vz = vel_cmd_msg->vel_cmd_z;
        const auto& dt = vel_cmd_msg->vel_cmd_dt;
        try
        {
            // Send command to server
            client_move_by_velocity(vx, vy, vz, dt, vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::local_pos_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::PosCmd::SharedPtr pos_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received local position command for vehicle %s", vehicle_name.c_str());

        if (!client_get_enabled_control(vehicle_name))
        {
            RCLCPP_ERROR(nh_->get_logger(), "Vehicle %s is not in OFFBOARD mode. Cannot send position command", vehicle_name.c_str());
            return;
        }

        // Extract message data
        const auto& x = pos_cmd_msg->pos_cmd_x;
        const auto& y = pos_cmd_msg->pos_cmd_y;
        const auto& z = pos_cmd_msg->pos_cmd_z;
        const auto& vel = pos_cmd_msg->pos_cmd_vel;
        const auto& timeout = pos_cmd_msg->pos_cmd_timeout;
        try
        {
            // Send command to server
            client_move_by_position(x, y, z, vel, timeout, vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::global_pos_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::PosCmd::SharedPtr pos_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received global position command for vehicle %s", vehicle_name.c_str());

        if (!client_get_enabled_control(vehicle_name))
        {
            RCLCPP_ERROR(nh_->get_logger(), "Vehicle %s is not in OFFBOARD mode. Cannot send global position command", vehicle_name.c_str());
            return;
        }

        // Extract message data
        const auto& x = pos_cmd_msg->pos_cmd_x;
        const auto& y = pos_cmd_msg->pos_cmd_y;
        const auto& z = pos_cmd_msg->pos_cmd_z;
        const auto& vel = pos_cmd_msg->pos_cmd_vel;
        const auto& timeout = pos_cmd_msg->pos_cmd_timeout;
        try
        {
            // Get global point
            geometry_msgs::msg::Point global_point;
            global_point.x = x;
            global_point.y = y;
            global_point.z = z;

            // Transform position to local frame
            const auto& local_point = transform_position_to_local(global_point, vehicle_map_[vehicle_name]->local_frame_id);

            // Send command to server
            client_move_by_position(local_point.x, local_point.y, local_point.z, vel, timeout, vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::gimbal_angle_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr gimbal_angle_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received gimbal angle command for vehicle %s", vehicle_name.c_str());

        // Extract message data
        const auto& camera_names = gimbal_angle_cmd_msg->camera_names;
        const auto& orientations = gimbal_angle_cmd_msg->orientations;
        try
        {
            for (size_t i = 0; i < camera_names.size(); i++)
            {
                const auto& camera_name = camera_names[i];
                const auto& orientation = orientations[i];

                // Convert orientation to airlib
                msr::airlib::Quaternionr attitude = get_gimbal_quat(orientation);

                // Send command to server
                client_set_gimbal_attitude(attitude, camera_name, vehicle_name);
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::camera_fov_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::CameraFovCmd::SharedPtr camera_fov_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received camera fov command for vehicle %s", vehicle_name.c_str());

        // Extract message data
        const auto& camera_names = camera_fov_cmd_msg->camera_names;
        const auto& fov_cmds = camera_fov_cmd_msg->fovs;
        try
        {
            for (size_t i = 0; i < camera_names.size(); i++)
            {
                const auto& camera_name = camera_names[i];
                const auto& fov_cmd = fov_cmds[i];

                // Send command to server
                client_set_camera_fov(camera_name, fov_cmd, vehicle_name);
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::window_image_cmd_group_cb(const airsim_interfaces::msg::WindowImageCmdGroup::SharedPtr window_image_cmd_group_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received window image command group");

        // Extract request data
        const auto& window_indices = window_image_cmd_group_msg->window_indices;
        const auto& vehicle_names = window_image_cmd_group_msg->vehicle_names;
        const auto& camera_names = window_image_cmd_group_msg->camera_names;
        const auto& corners = window_image_cmd_group_msg->corners;
        const auto& sizes = window_image_cmd_group_msg->sizes;
        try
        {
            // Send command to server
            client_set_window_images(window_indices, vehicle_names, camera_names, get_airlib_points_2d(corners), get_airlib_points_2d(sizes));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::window_rectangle_cmd_cb(const airsim_interfaces::msg::WindowRectangleCmd::SharedPtr window_rectangle_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received window rectangle command group");

        // Extract request data
        const auto& window_index = window_rectangle_cmd_msg->window_index;
        const auto& corners = window_rectangle_cmd_msg->corners;
        const auto& sizes = window_rectangle_cmd_msg->sizes;
        const auto& color = window_rectangle_cmd_msg->color;
        const auto& thickness = window_rectangle_cmd_msg->thickness;
        try
        {
            // Send command to server
            client_set_window_rectangles(window_index, get_airlib_points_2d(corners), get_airlib_points_2d(sizes), get_airlib_color(color), thickness);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::window_string_cmd_cb(const airsim_interfaces::msg::WindowStringCmd::SharedPtr window_string_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received window string command group");

        // Extract request data
        const auto& window_index = window_string_cmd_msg->window_index;
        const auto& strings = window_string_cmd_msg->strings;
        const auto& positions = window_string_cmd_msg->positions;
        const auto& color = window_string_cmd_msg->color;
        const auto& scale = window_string_cmd_msg->scale;
        try
        {
            // Send command to server
            client_set_window_strings(window_index, strings, get_airlib_points_2d(positions), get_airlib_color(color), scale);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::update_target_cmd_group_cb(const airsim_interfaces::msg::UpdateTargetCmdGroup::SharedPtr update_target_cmd_group_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received update target command group");

        // Extract message data
        const auto& target_names = update_target_cmd_group_msg->target_names;
        const auto& positions = update_target_cmd_group_msg->positions;
        try
        {
            // Send command to server
            client_update_targets(target_names, get_airlib_points(positions));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::update_cluster_cmd_group_cb(const airsim_interfaces::msg::UpdateClusterCmdGroup::SharedPtr update_cluster_cmd_group_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received update cluster command group");

        // Extract message data
        const auto& cluster_names = update_cluster_cmd_group_msg->cluster_names;
        const auto& centers = update_cluster_cmd_group_msg->centers;
        const auto& radii = update_cluster_cmd_group_msg->radii;
        try
        {
            // Send command to server
            client_update_clusters(cluster_names, get_airlib_points(centers), radii);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // COMMAND SERVICES
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimWrapper::load_level_srv_cb(std::shared_ptr<airsim_interfaces::srv::LoadLevel::Request> request, std::shared_ptr<airsim_interfaces::srv::LoadLevel::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Loading level %s", request->level_name.c_str());
        const auto& level_name = request->level_name;

        // Load level
        try
        {
            // Send command to server
            client_load_level(level_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::reset_srv_cb(std::shared_ptr<airsim_interfaces::srv::Reset::Request> request, std::shared_ptr<airsim_interfaces::srv::Reset::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Resetting AirSim");
        unused(request);

        // Pause and reset AirSim
        try
        {
            // Send command to server
            client_reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            client_pause(true);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::run_srv_cb(std::shared_ptr<airsim_interfaces::srv::Run::Request> request, std::shared_ptr<airsim_interfaces::srv::Run::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Running AirSim");
        unused(request);

        try
        {
            // Send command to server
            client_pause(false);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::pause_srv_cb(std::shared_ptr<airsim_interfaces::srv::Pause::Request> request, std::shared_ptr<airsim_interfaces::srv::Pause::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Pausing AirSim");
        unused(request);

        try
        {
            // Send command to server
            client_pause(true);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::enable_control_srv_cb(std::shared_ptr<airsim_interfaces::srv::EnableControl::Request> request, std::shared_ptr<airsim_interfaces::srv::EnableControl::Response> response)
    {
        const auto& vehicle_name = request->vehicle_name;
        const auto& enable = request->enable;

        if (enable)
        {
            RCLCPP_INFO(nh_->get_logger(), "Enabling control of vehicle %s", vehicle_name.c_str());
        }
        else
        {
            RCLCPP_INFO(nh_->get_logger(), "Disabling control of vehicle %s", vehicle_name.c_str());
        }

        try
        {
            // Send command to server
            client_enable_control(enable, vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::arm_disarm_srv_cb(std::shared_ptr<airsim_interfaces::srv::ArmDisarm::Request> request, std::shared_ptr<airsim_interfaces::srv::ArmDisarm::Response> response)
    {
        const auto& vehicle_name = request->vehicle_name;
        const auto& arm = request->arm;

        if (arm)
        {
            RCLCPP_INFO(nh_->get_logger(), "Arming vehicle %s", vehicle_name.c_str());
        }
        else
        {
            RCLCPP_INFO(nh_->get_logger(), "Disarming vehicle %s", vehicle_name.c_str());
        }

        try
        {
            // Send command to server
            client_arm_disarm(arm, vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::takeoff_srv_cb(std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request, std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
    {
        const auto& vehicle_name = request->vehicle_name;
        RCLCPP_INFO(nh_->get_logger(), "Taking off vehicle %s", vehicle_name.c_str());

        try
        {
            // Send command to server
            client_takeoff(20, vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::land_srv_cb(std::shared_ptr<airsim_interfaces::srv::Land::Request> request, std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
    {
        const auto& vehicle_name = request->vehicle_name;
        RCLCPP_INFO(nh_->get_logger(), "Landing vehicle %s", vehicle_name.c_str());

        try
        {
            // Send command to server
            client_land(60, vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::hover_srv_cb(std::shared_ptr<airsim_interfaces::srv::Hover::Request> request, std::shared_ptr<airsim_interfaces::srv::Hover::Response> response)
    {
        const auto& vehicle_name = request->vehicle_name;
        RCLCPP_INFO(nh_->get_logger(), "Hovering vehicle %s", vehicle_name.c_str());

        try
        {
            // Send command to server
            client_hover(vehicle_name);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::camera_capture_srv_cb(std::shared_ptr<airsim_interfaces::srv::CameraCapture::Request> request, std::shared_ptr<airsim_interfaces::srv::CameraCapture::Response> response)
    {
        const auto& vehicle_name = request->vehicle_name;
        const auto& active = request->active;

        if (active)
        {
            RCLCPP_INFO(nh_->get_logger(), "Activating camera capture for vehicle %s", vehicle_name.c_str());
        }
        else
        {
            RCLCPP_INFO(nh_->get_logger(), "Deactivating camera capture for vehicle %s", vehicle_name.c_str());
        }

        try
        {
            response->success = client_set_agent_cameras_active(vehicle_name, active);
            return response->success;
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }
    }

    bool AirsimWrapper::add_target_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Adding group of targets");

        // Extract request data
        const auto& target_names = request->target_names;
        const auto& target_types = request->target_types;
        const auto& positions = request->positions;
        const auto& highlight = request->highlight;
        const auto& highlight_color_rgba = request->highlight_color_rgba;
        try
        {
            // Send command to server
            client_add_targets(target_names, target_types, get_airlib_points(positions), highlight, get_airlib_colors(highlight_color_rgba));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::add_cluster_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Adding group of clusters");

        // Extract request data
        const auto& cluster_names = request->cluster_names;
        const auto& centers = request->centers;
        const auto& radii = request->radii;
        const auto& highlight = request->highlight;
        const auto& highlight_color_rgba = request->highlight_color_rgba;
        try
        {
            // Send command to server
            client_add_clusters(cluster_names, get_airlib_points(centers), radii, highlight, get_airlib_colors(highlight_color_rgba));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::remove_all_targets_cb(const std::shared_ptr<airsim_interfaces::srv::RemoveAllTargets::Request> request, const std::shared_ptr<airsim_interfaces::srv::RemoveAllTargets::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Removing all targets");
        unused(request);

        try
        {
            // Send command to server
            client_remove_all_targets();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::remove_all_clusters_cb(const std::shared_ptr<airsim_interfaces::srv::RemoveAllClusters::Request> request, const std::shared_ptr<airsim_interfaces::srv::RemoveAllClusters::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Removing all clusters");
        unused(request);

        try
        {
            // Send command to server
            client_remove_all_clusters();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // TIMER CALLBACKS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::state_timer_cb()
    {
        try
        {
            // Retrieve simulation state
            bool is_paused;
            rclcpp::Time curr_time;
            is_paused = client_get_paused();
            curr_time = get_sim_clock_time();

            // Update simulation state if not paused
            if (!is_paused)
            {
                // Iterate over vehicles
                for (auto& [vehicle_name, vehicle_ros] : vehicle_map_)
                {
                    // Retrieve vehicle state from server
                    msr::airlib::MultirotorState multirotor_state;
                    multirotor_state = client_get_multirotor_state(vehicle_name);

                    // Update vehicle odom and tf
                    vehicle_ros->local_odom.header.stamp = curr_time;
                    vehicle_ros->global_odom.header.stamp = curr_time;
                    vehicle_ros->body_tf_msg.header.stamp = curr_time;
                    update_vehicle_odom(vehicle_ros.get(), multirotor_state.kinematics_estimated);
                    vehicle_ros->body_tf_msg.transform = get_transform_msg_from_airsim(multirotor_state.getPosition(), multirotor_state.getOrientation());

                    // Debug: Get ground truth position and compare with estimated position
                    // msr::airlib::Pose ground_truth_pose = client_get_ground_truth_multirotor_state(vehicle_name).kinematics_estimated.pose;
                    // RCLCPP_INFO(nh_->get_logger(), "Ground truth vs estimated position: %f, %f, %f",
                    //     ground_truth_pose.position.x() - multirotor_state.kinematics_estimated.pose.position.x(),
                    //     ground_truth_pose.position.y() - multirotor_state.kinematics_estimated.pose.position.y(),
                    //     ground_truth_pose.position.z() - multirotor_state.kinematics_estimated.pose.position.z());

                    // Iterate over cameras
                    vehicle_ros->camera_orientation.header.stamp = curr_time;
                    vehicle_ros->camera_orientation.camera_names.clear();
                    vehicle_ros->camera_orientation.orientations.clear();
                    for (auto& [camera_name, camera_ros] : vehicle_ros->camera_map)
                    {
                        // Get camera pose
                        const auto& pose = client_get_camera_pose(vehicle_ros->vehicle_name, camera_ros->camera_name);

                        // Update camera body tf
                        camera_ros->body_tf_msg.header.stamp = curr_time;
                        camera_ros->body_tf_msg.transform = get_gimbal_transform_msg_from_airsim(pose.position, pose.orientation);

                        // Update camera orientation
                        tf2::Quaternion camera_orientation_tf2;
                        tf2::fromMsg(camera_ros->body_tf_msg.transform.rotation, camera_orientation_tf2);
                        geometry_msgs::msg::Quaternion camera_orientation;
                        camera_orientation.x = camera_orientation_tf2.x();
                        camera_orientation.y = camera_orientation_tf2.y();
                        camera_orientation.z = camera_orientation_tf2.z();
                        camera_orientation.w = camera_orientation_tf2.w();
                        vehicle_ros->camera_orientation.camera_names.push_back(camera_name);
                        vehicle_ros->camera_orientation.orientations.push_back(camera_orientation);
                    }
                }
            }

            // Publish simulation state
            publish_vehicle_state();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::clock_timer_cb()
    {
        try
        {
            // Retrieve timestamp from server
            {
                std::lock_guard<std::mutex> lock(clock_mutex_);
                ros_clock_.clock = client_get_timestamp();
            }

            // Publish clock
            clock_pub_->publish(ros_clock_);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::update_vehicle_odom(VehicleROS* vehicle_ros, const msr::airlib::Kinematics::State& kinematics_estimated)
    {
        // Get estimated local pose and twist
        auto& est_local_pose = kinematics_estimated.pose;
        auto& est_local_twist = kinematics_estimated.twist;

        // Get odom pose and twist
        auto& local_pose = vehicle_ros->local_odom.pose.pose;
        auto& local_twist = vehicle_ros->local_odom.twist.twist;
        auto& global_pose = vehicle_ros->global_odom.pose.pose;
        auto& global_twist = vehicle_ros->global_odom.twist.twist;

        // Update local odom pose
        local_pose.position.x = est_local_pose.position.x();
        local_pose.position.y = -est_local_pose.position.y();
        local_pose.position.z = -est_local_pose.position.z();
        local_pose.orientation.x = est_local_pose.orientation.x();
        local_pose.orientation.y = -est_local_pose.orientation.y();
        local_pose.orientation.z = -est_local_pose.orientation.z();
        local_pose.orientation.w = est_local_pose.orientation.w();

        // Update local odom twist
        local_twist.linear.x = est_local_twist.linear.x();
        local_twist.linear.y = -est_local_twist.linear.y();
        local_twist.linear.z = -est_local_twist.linear.z();
        local_twist.angular.x = est_local_twist.angular.x();
        local_twist.angular.y = -est_local_twist.angular.y();
        local_twist.angular.z = -est_local_twist.angular.z();

        // Update global pose and twist
        global_pose = transform_pose_to_global(local_pose, vehicle_ros->local_frame_id);
        global_twist = transform_twist_to_global(local_twist, vehicle_ros->local_frame_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLISH FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::publish_vehicle_state()
    {
        for (auto& [vehicle_name, vehicle_ros] : vehicle_map_)
        {
            // Publish vehicle odom
            vehicle_ros->local_odom_pub->publish(vehicle_ros->local_odom);
            vehicle_ros->global_odom_pub->publish(vehicle_ros->global_odom);
            vehicle_ros->camera_orientation_pub->publish(vehicle_ros->camera_orientation);

            // Publish vehicle odom tf
            if (broadcast_transforms_)
            {
                tf_broadcaster_->sendTransform(vehicle_ros->body_tf_msg);
            }

            // Publish camera body tf
            for (auto& [_, camera_ros] : vehicle_ros->camera_map)
            {
                if (broadcast_transforms_)
                {
                    tf_broadcaster_->sendTransform(camera_ros->body_tf_msg);
                }
            }
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // AIRSIM CLIENT FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    rclcpp::Time AirsimWrapper::client_get_timestamp()
    {
        return rclcpp::Time(airsim_client_clock_->getTimestamp());
    }

    bool AirsimWrapper::client_get_paused()
    {
        return airsim_client_state_->simIsPaused();
    }

    bool AirsimWrapper::client_get_enabled_control(const std::string& vehicle_name)
    {
        return airsim_client_state_->isApiControlEnabled(vehicle_name);
    }

    msr::airlib::MultirotorState AirsimWrapper::client_get_multirotor_state(const std::string& vehicle_name)
    {
        return airsim_client_state_->getMultirotorState(vehicle_name);
    }

    msr::airlib::MultirotorState AirsimWrapper::client_get_ground_truth_multirotor_state(const std::string& vehicle_name)
    {
        msr::airlib::MultirotorState multirotor_state;
        multirotor_state.kinematics_estimated.pose = airsim_client_state_->simGetVehiclePose(vehicle_name);
        return multirotor_state;
    }

    msr::airlib::Pose AirsimWrapper::client_get_camera_pose(const std::string& vehicle_name, const std::string& gimbal_name)
    {
        return airsim_client_state_->getCameraPose(gimbal_name, vehicle_name);
    }

    void AirsimWrapper::client_load_level(const std::string& level_name)
    {
        airsim_client_control_->simLoadLevel(level_name);
    }

    void AirsimWrapper::client_reset()
    {
        airsim_client_control_->reset();
    }

    void AirsimWrapper::client_pause(const bool& is_paused)
    {
        airsim_client_control_->simPause(is_paused);
    }

    void AirsimWrapper::client_enable_control(const bool& enable, const std::string& vehicle_name)
    {
        airsim_client_control_->enableApiControl(enable, vehicle_name);
    }

    void AirsimWrapper::client_arm_disarm(const bool& arm, const std::string& vehicle_name)
    {
        airsim_client_control_->armDisarm(arm, vehicle_name);
    }

    void AirsimWrapper::client_takeoff(const float& timeout, const std::string& vehicle_name, const bool& wait)
    {
        if (wait)
        {
            airsim_client_control_->takeoffAsync(timeout, vehicle_name)->waitOnLastTask();
        }
        else
        {
            airsim_client_control_->takeoffAsync(timeout, vehicle_name);
        }
    }

    void AirsimWrapper::client_land(const float& timeout, const std::string& vehicle_name, const bool& wait)
    {
        if (wait)
        {
            airsim_client_control_->landAsync(timeout, vehicle_name)->waitOnLastTask();
        }
        else
        {
            airsim_client_control_->landAsync(timeout, vehicle_name);
        }
    }
    void AirsimWrapper::client_hover(const std::string& vehicle_name)
    {
        airsim_client_control_->hoverAsync(vehicle_name);
    }

    void AirsimWrapper::client_move_by_velocity(const float& vx, const float& vy, const float& vz, const float& dt, const std::string& vehicle_name)
    {
        airsim_client_control_->moveByVelocityAsync(vx, -vy, -vz, dt,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(), vehicle_name);
    }

    void AirsimWrapper::client_move_by_position(const float& x, const float& y, const float& z, const float& vel, const float& timeout, const std::string& vehicle_name)
    {
        airsim_client_control_->moveToPositionAsync(x, -y, -z, vel, timeout,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(),
            -1, 1, vehicle_name);
    }

    void AirsimWrapper::client_set_gimbal_attitude(const msr::airlib::Quaternionr& attitude, const std::string& camera_name, const std::string& vehicle_name)
    {
        airsim_client_control_->setGimbalAttitude(attitude, camera_name, vehicle_name);
    }

    void AirsimWrapper::client_set_camera_fov(const std::string& camera_name, const float& fov, const std::string& vehicle_name)
    {
        airsim_client_control_->simSetCameraFov(camera_name, math_common::rad2deg(fov), vehicle_name);
    }

    bool AirsimWrapper::client_set_agent_cameras_active(const std::string& vehicle_name, const bool& active)
    {
        return airsim_client_control_->simSetAgentCamerasActive(vehicle_name, active);
    }

    void AirsimWrapper::client_set_window_images(const std::vector<int>& window_indices, const std::vector<std::string>& vehicle_names, const std::vector<std::string>& camera_names, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes)
    {
        airsim_client_window_->simSetWindowImages(window_indices, vehicle_names, camera_names, corners, sizes);
    }

    void AirsimWrapper::client_set_window_rectangles(const int& window_index, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes, const std::vector<float>& color, const float& thickness)
    {
        airsim_client_window_->simDrawRectangles(window_index, corners, sizes, color, thickness);
    }

    void AirsimWrapper::client_set_window_strings(const int& window_index, const std::vector<std::string>& strings, const std::vector<msr::airlib::Vector2r>& positions, const std::vector<float>& color, const float& scale)
    {
        airsim_client_window_->simDrawStrings(window_index, strings, positions, color, scale);
    }

    void AirsimWrapper::client_add_targets(const std::vector<std::string>& target_names, const std::vector<std::string>& target_types, const std::vector<msr::airlib::Vector3r>& positions, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba)
    {
        airsim_client_tracking_->simAddTargets(target_names, target_types, positions, highlight, highlight_color_rgba);
    }

    void AirsimWrapper::client_add_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba)
    {
        airsim_client_tracking_->simAddClusters(cluster_names, centers, radii, highlight, highlight_color_rgba);
    }

    void AirsimWrapper::client_remove_targets(const std::vector<std::string>& target_names)
    {
        airsim_client_tracking_->simRemoveTargets(target_names);
    }

    void AirsimWrapper::client_remove_clusters(const std::vector<std::string>& cluster_names)
    {
        airsim_client_tracking_->simRemoveClusters(cluster_names);
    }

    void AirsimWrapper::client_remove_all_targets()
    {
        airsim_client_tracking_->simRemoveAllTargets();
    }

    void AirsimWrapper::client_remove_all_clusters()
    {
        airsim_client_tracking_->simRemoveAllClusters();
    }

    void AirsimWrapper::client_update_targets(const std::vector<std::string>& target_names, const std::vector<msr::airlib::Vector3r>& positions)
    {
        airsim_client_tracking_->simUpdateTargets(target_names, positions);
    }

    void AirsimWrapper::client_update_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii)
    {
        airsim_client_tracking_->simUpdateClusters(cluster_names, centers, radii);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UTILITY FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    rclcpp::Time AirsimWrapper::get_sim_clock_time()
    {
        std::lock_guard<std::mutex> lock(clock_mutex_);
        return ros_clock_.clock;
    }

    msr::airlib::Quaternionr AirsimWrapper::get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
    {
        return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, -geometry_msgs_quat.y, -geometry_msgs_quat.z);
    }

    msr::airlib::Quaternionr AirsimWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
    {
        return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), -tf2_quat.y(), -tf2_quat.z());
    }

    msr::airlib::Pose AirsimWrapper::get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const
    {
        return msr::airlib::Pose(msr::airlib::Vector3r(x, -y, -z), airlib_quat);
    }

    msr::airlib::Pose AirsimWrapper::get_airlib_pose(const geometry_msgs::msg::Pose& geometry_msgs_pose) const
    {
        return get_airlib_pose(geometry_msgs_pose.position.x, -geometry_msgs_pose.position.y, -geometry_msgs_pose.position.z, get_airlib_quat(geometry_msgs_pose.orientation));
    }

    msr::airlib::Vector3r AirsimWrapper::get_airlib_rpy(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
    {
        msr::airlib::Quaternionr quat = get_airlib_quat(geometry_msgs_quat);
        return quat.toRotationMatrix().eulerAngles(0, 1, 2);
    }

    msr::airlib::Vector3r AirsimWrapper::get_airlib_point(const geometry_msgs::msg::Point& geometry_msgs_point) const
    {
        return msr::airlib::Vector3r(geometry_msgs_point.x, -geometry_msgs_point.y, -geometry_msgs_point.z);
    }

    msr::airlib::Vector2r AirsimWrapper::get_airlib_point_2d(const geometry_msgs::msg::Point& geometry_msgs_point) const
    {
        return msr::airlib::Vector2r(geometry_msgs_point.x, geometry_msgs_point.y);
    }

    std::vector<msr::airlib::Vector3r> AirsimWrapper::get_airlib_points(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const
    {
        std::vector<msr::airlib::Vector3r> airlib_points(geometry_msgs_points.size());
        for (size_t i = 0; i < geometry_msgs_points.size(); i++)
        {
            airlib_points[i] = get_airlib_point(geometry_msgs_points[i]);
        }
        return airlib_points;
    }

    std::vector<msr::airlib::Vector2r> AirsimWrapper::get_airlib_points_2d(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const
    {
        std::vector<msr::airlib::Vector2r> airlib_points(geometry_msgs_points.size());
        for (size_t i = 0; i < geometry_msgs_points.size(); i++)
        {
            airlib_points[i] = get_airlib_point_2d(geometry_msgs_points[i]);
        }
        return airlib_points;
    }

    std::vector<float> AirsimWrapper::get_airlib_color(const std_msgs::msg::ColorRGBA& std_msgs_color) const
    {
        return std::vector<float>{std_msgs_color.r, std_msgs_color.g, std_msgs_color.b, std_msgs_color.a};
    }

    std::vector<std::vector<float>> AirsimWrapper::get_airlib_colors(const std::vector<std_msgs::msg::ColorRGBA>& std_msgs_colors) const
    {
        std::vector<std::vector<float>> airlib_colors;
        for (const auto& std_msgs_color : std_msgs_colors)
        {
            airlib_colors.push_back(get_airlib_color(std_msgs_color));
        }
        return airlib_colors;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
    {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = position.x();
        transform.translation.y = -position.y();
        transform.translation.z = -position.z();
        tf2::Quaternion quat;
        quat.setRPY(rotation.roll * (M_PI / 180.0), rotation.pitch * (M_PI / 180.0), rotation.yaw * (M_PI / 180.0));
        transform.rotation.x = quat.x();
        transform.rotation.y = -quat.y();
        transform.rotation.z = -quat.z();
        transform.rotation.w = quat.w();

        return transform;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
    {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = position.x();
        transform.translation.y = -position.y();
        transform.translation.z = -position.z();
        transform.rotation.x = quaternion.x();
        transform.rotation.y = -quaternion.y();
        transform.rotation.z = -quaternion.z();
        transform.rotation.w = quaternion.w();

        return transform;
    }

    geometry_msgs::msg::Pose AirsimWrapper::get_pose_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = -position.y();
        pose.position.z = -position.z();
        tf2::Quaternion quat;
        quat.setRPY(rotation.roll * (M_PI / 180.0), rotation.pitch * (M_PI / 180.0), rotation.yaw * (M_PI / 180.0));
        pose.orientation.x = quat.x();
        pose.orientation.y = -quat.y();
        pose.orientation.z = -quat.z();
        pose.orientation.w = quat.w();

        return pose;
    }

    geometry_msgs::msg::Pose AirsimWrapper::get_pose_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = -position.y();
        pose.position.z = -position.z();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = -quaternion.y();
        pose.orientation.z = -quaternion.z();
        pose.orientation.w = quaternion.w();

        return pose;
    }

    // airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
    void AirsimWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
    {
        if (std::isnan(vehicle_setting.position.x()))
            vehicle_setting.position.x() = 0.0;

        if (std::isnan(vehicle_setting.position.y()))
            vehicle_setting.position.y() = 0.0;

        if (std::isnan(vehicle_setting.position.z()))
            vehicle_setting.position.z() = 0.0;

        if (std::isnan(vehicle_setting.rotation.yaw))
            vehicle_setting.rotation.yaw = 0.0;

        if (std::isnan(vehicle_setting.rotation.pitch))
            vehicle_setting.rotation.pitch = 0.0;

        if (std::isnan(vehicle_setting.rotation.roll))
            vehicle_setting.rotation.roll = 0.0;
    }

    // if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
    void AirsimWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
    {
        if (std::isnan(camera_setting.position.x()))
            camera_setting.position.x() = vehicle_setting.position.x();

        if (std::isnan(camera_setting.position.y()))
            camera_setting.position.y() = vehicle_setting.position.y();

        if (std::isnan(camera_setting.position.z()))
            camera_setting.position.z() = vehicle_setting.position.z();

        if (std::isnan(camera_setting.rotation.yaw))
            camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

        if (std::isnan(camera_setting.rotation.pitch))
            camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

        if (std::isnan(camera_setting.rotation.roll))
            camera_setting.rotation.roll = vehicle_setting.rotation.roll;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_camera_optical_tf() const
    {
        geometry_msgs::msg::Transform optical_tf = geometry_msgs::msg::Transform(); // zero translation
        auto opticalQ = msr::airlib::Quaternionr(optical_tf.rotation.w, optical_tf.rotation.x, optical_tf.rotation.y, optical_tf.rotation.z);
        opticalQ *= msr::airlib::Quaternionr(0.5, -0.5, 0.5, -0.5);
        optical_tf.rotation.w = opticalQ.w();
        optical_tf.rotation.x = opticalQ.x();
        optical_tf.rotation.y = opticalQ.y();
        optical_tf.rotation.z = opticalQ.z();
        return optical_tf;
    }

    geometry_msgs::msg::Point AirsimWrapper::transform_position_to_local(const geometry_msgs::msg::Point& global_point, const std::string& local_frame) const
    {
        // Stamp the point
        geometry_msgs::msg::PointStamped point_stamped_msg;
        point_stamped_msg.header.frame_id = world_frame_id_;
        point_stamped_msg.header.stamp = rclcpp::Time(0);
        point_stamped_msg.point = global_point;

        // Transform the point
        try
        {
            return tf_buffer_->transform(point_stamped_msg, local_frame).point;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(nh_->get_logger(), "Transform failed: %s", ex.what());
            return global_point;  // Return original point on failure
        }
    }

    geometry_msgs::msg::Pose AirsimWrapper::transform_pose_to_global(const geometry_msgs::msg::Pose& local_pose, const std::string& local_frame) const
    {
        // Stamp the pose
        geometry_msgs::msg::PoseStamped pose_stamped_msg;
        pose_stamped_msg.header.frame_id = local_frame;
        pose_stamped_msg.header.stamp = rclcpp::Time(0);
        pose_stamped_msg.pose = local_pose;

        // Transform the pose
        try
        {
            return tf_buffer_->transform(pose_stamped_msg, world_frame_id_).pose;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(nh_->get_logger(), "Transform failed: %s", ex.what());
            return local_pose;  // Return original pose on failure
        }
    }

    geometry_msgs::msg::Twist AirsimWrapper::transform_twist_to_global(const geometry_msgs::msg::Twist& local_twist, const std::string& local_frame) const
    {
        // Get the transform between the source and target frames
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(world_frame_id_, local_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(nh_->get_logger(), "Transform failed: %s", ex.what());
            return local_twist;  // Return original twist on failure
        }

        // Convert to tf2 types for transformation
        tf2::Vector3 tf_linear(local_twist.linear.x, local_twist.linear.y, local_twist.linear.z);
        tf2::Vector3 tf_angular(local_twist.angular.x, local_twist.angular.y, local_twist.angular.z);

        // Create transform object
        tf2::Quaternion rotation;
        tf2::fromMsg(transform.transform.rotation, rotation);

        // Rotate vectors
        tf2::Vector3 tf_linear_rotated = tf2::quatRotate(rotation, tf_linear);
        tf2::Vector3 tf_angular_rotated = tf2::quatRotate(rotation, tf_angular);

        // Pack results back into TwistMsg
        geometry_msgs::msg::Twist transformed_twist_msg;
        transformed_twist_msg.linear.x = tf_linear_rotated.x();
        transformed_twist_msg.linear.y = tf_linear_rotated.y();
        transformed_twist_msg.linear.z = tf_linear_rotated.z();
        transformed_twist_msg.angular.x = tf_angular_rotated.x();
        transformed_twist_msg.angular.y = tf_angular_rotated.y();
        transformed_twist_msg.angular.z = tf_angular_rotated.z();

        return transformed_twist_msg;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_gimbal_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
    {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = position.x();
        transform.translation.y = -position.y();
        transform.translation.z = -position.z();
        transform.rotation.x = quaternion.x();
        transform.rotation.y = -quaternion.y();
        transform.rotation.z = -quaternion.z();
        transform.rotation.w = quaternion.w();

        // Rotate the transform by 90 degrees around the y-axis
        tf2::Quaternion quat, quat_gimbal, quat_final;
        quat.setRPY(0.0, M_PI_2, 0.0);
        tf2::fromMsg(transform.rotation, quat_gimbal);
        quat_final = quat_gimbal * quat;
        transform.rotation.x = quat_final.x();
        transform.rotation.y = quat_final.y();
        transform.rotation.z = quat_final.z();
        transform.rotation.w = quat_final.w();

        return transform;
    }

    msr::airlib::Quaternionr AirsimWrapper::get_gimbal_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
    {
        // Rotate the quaternion by -90 degrees around the y-axis
        tf2::Quaternion quat, quat_gimbal, quat_final;
        quat.setRPY(0.0, -M_PI_2, 0.0);
        tf2::fromMsg(geometry_msgs_quat, quat_gimbal);
        quat_final = quat_gimbal * quat;

        return msr::airlib::Quaternionr(quat_final.w(), quat_final.x(), -quat_final.y(), -quat_final.z());
    }

} // namespace airsim_wrapper