#include <airsim_wrapper.h>

using namespace std::placeholders;

namespace airsim_wrapper
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR/DESTRUCTOR
    // ════════════════════════════════════════════════════════════════════════════

    AirsimWrapper::AirsimWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip, uint16_t host_port)
        : airsim_settings_parser_(host_ip, host_port)
        , host_ip_(host_ip)
        , host_port_(host_port)
        , airsim_client_control_(nullptr)
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

        // Create clock timer
        nh_->get_parameter("update_sim_clock_every_n_sec", update_sim_clock_every_n_sec_);
        sim_clock_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_sim_clock_every_n_sec_), std::bind(&AirsimWrapper::clock_timer_cb, this));
        RCLCPP_INFO(nh_->get_logger(), "Created clock timer (%.4f)", update_sim_clock_every_n_sec_);

        RCLCPP_INFO(nh_->get_logger(), "AirsimWrapper successfully initialized!");
    }

    AirsimWrapper::~AirsimWrapper()
    {
        shutdown();
    }

    void AirsimWrapper::shutdown()
    {
        sim_clock_update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::initialize_airsim()
    {
        airsim_client_control_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        airsim_client_tracking_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::RpcLibClientBase(host_ip_, host_port_));
        airsim_client_clock_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));

        std::vector<std::future<void>> futures;
        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_control_->confirmConnection();
        }));
        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_tracking_->confirmConnection();
        }));
        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_clock_->confirmConnection();
        }));
        for (auto& future : futures)
        {
            future.get();
        }

        ros_clock_.clock = client_get_timestamp();
    }

    void AirsimWrapper::initialize_ros()
    {
        // Create ROS communications
        RCLCPP_INFO(nh_->get_logger(), "Creating ROS communications...");
        clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
        // Global simulation control services
        reset_srvr_ = nh_->create_service<airsim_interfaces::srv::Reset>("reset", std::bind(&AirsimWrapper::reset_srv_cb, this, _1, _2));
        run_srvr_ = nh_->create_service<airsim_interfaces::srv::Run>("run", std::bind(&AirsimWrapper::run_srv_cb, this, _1, _2));
        pause_srvr_ = nh_->create_service<airsim_interfaces::srv::Pause>("pause", std::bind(&AirsimWrapper::pause_srv_cb, this, _1, _2));
        // Vehicle camera service
        camera_capture_srvr_ = nh_->create_service<airsim_interfaces::srv::CameraCapture>("vehicles/cmd/camera_capture", std::bind(&AirsimWrapper::camera_capture_srv_cb, this, _1, _2));
        // Vehicle camera subscribers
        gimbal_angle_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleCmd>(
            "vehicles/cmd/gimbal_angle", 10, std::bind(&AirsimWrapper::gimbal_angle_cmd_cb, this, _1));
        camera_fov_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::CameraFovCmd>(
            "vehicles/cmd/camera_fov", 10, std::bind(&AirsimWrapper::camera_fov_cmd_cb, this, _1));
        // Tracking services
        add_target_group_srvr_ = nh_->create_service<airsim_interfaces::srv::AddTargetGroup>("targets/cmd/add", std::bind(&AirsimWrapper::add_target_group_cb, this, _1, _2));
        add_cluster_group_srvr_ = nh_->create_service<airsim_interfaces::srv::AddClusterGroup>("clusters/cmd/add", std::bind(&AirsimWrapper::add_cluster_group_cb, this, _1, _2));
        remove_all_targets_srvr_ = nh_->create_service<airsim_interfaces::srv::RemoveAllTargets>("targets/cmd/remove_all", std::bind(&AirsimWrapper::remove_all_targets_cb, this, _1, _2));
        remove_all_clusters_srvr_ = nh_->create_service<airsim_interfaces::srv::RemoveAllClusters>("clusters/cmd/remove_all", std::bind(&AirsimWrapper::remove_all_clusters_cb, this, _1, _2));
        // Tracking subscribers
        update_target_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::UpdateTargetCmdGroup>(
            "targets/cmd/update", 10, std::bind(&AirsimWrapper::update_target_cmd_group_cb, this, _1));
        update_cluster_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::UpdateClusterCmdGroup>(
            "clusters/cmd/update", 10, std::bind(&AirsimWrapper::update_cluster_cmd_group_cb, this, _1));
    }

    // ════════════════════════════════════════════════════════════════════════════
    // COMMAND CALLBACKS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::gimbal_angle_cmd_cb(const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr gimbal_angle_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received gimbal angle command for vehicle %s", gimbal_angle_cmd_msg->vehicle_name.c_str());

        const auto& vehicle_name = gimbal_angle_cmd_msg->vehicle_name;
        const auto& camera_names = gimbal_angle_cmd_msg->camera_names;
        const auto& orientations = gimbal_angle_cmd_msg->orientations;
        try
        {
            for (size_t i = 0; i < camera_names.size(); i++)
            {
                client_set_gimbal_attitude(get_gimbal_quat(orientations[i]), camera_names[i], vehicle_name);
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::camera_fov_cmd_cb(const airsim_interfaces::msg::CameraFovCmd::SharedPtr camera_fov_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received camera fov command for vehicle %s", camera_fov_cmd_msg->vehicle_name.c_str());

        const auto& vehicle_name = camera_fov_cmd_msg->vehicle_name;
        const auto& camera_names = camera_fov_cmd_msg->camera_names;
        const auto& fov_cmds = camera_fov_cmd_msg->fovs;
        try
        {
            for (size_t i = 0; i < camera_names.size(); i++)
            {
                client_set_camera_fov(camera_names[i], fov_cmds[i], vehicle_name);
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
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
    // AIRSIM CLIENT FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    rclcpp::Time AirsimWrapper::client_get_timestamp()
    {
        return rclcpp::Time(airsim_client_clock_->getTimestamp());
    }

    void AirsimWrapper::client_reset()
    {
        airsim_client_control_->reset();
    }

    void AirsimWrapper::client_pause(const bool& is_paused)
    {
        airsim_client_control_->simPause(is_paused);
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

    void AirsimWrapper::client_add_targets(const std::vector<std::string>& target_names, const std::vector<std::string>& target_types, const std::vector<msr::airlib::Vector3r>& positions, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba)
    {
        airsim_client_tracking_->simAddTargets(target_names, target_types, positions, highlight, highlight_color_rgba);
    }

    void AirsimWrapper::client_add_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba)
    {
        airsim_client_tracking_->simAddClusters(cluster_names, centers, radii, highlight, highlight_color_rgba);
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

    msr::airlib::Vector3r AirsimWrapper::get_airlib_point(const geometry_msgs::msg::Point& geometry_msgs_point) const
    {
        return msr::airlib::Vector3r(geometry_msgs_point.x, -geometry_msgs_point.y, -geometry_msgs_point.z);
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