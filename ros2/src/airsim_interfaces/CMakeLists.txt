cmake_minimum_required(VERSION 3.8)
project(airsim_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Global services and messages
  "srv/LoadLevel.srv"
  "srv/Reset.srv"
  "srv/Run.srv"
  "srv/Pause.srv"
  # Vehicle services and messages
  "srv/Takeoff.srv"
  "srv/Land.srv"
  "srv/Hover.srv"
  "srv/EnableControl.srv"
  "srv/ArmDisarm.srv"
  "srv/CameraCapture.srv"
  "msg/VelCmd.msg"
  "msg/PosCmd.msg"
  "msg/CameraFovCmd.msg"
  "msg/GimbalAngleCmd.msg"
  "msg/CameraOrientation.msg"
  # Window services and messages
  "msg/WindowImageCmdGroup.msg"
  "msg/WindowRectangleCmd.msg"
  "msg/WindowStringCmd.msg"
  # Tracking services and messages
  "srv/AddTargetGroup.srv"
  "srv/AddClusterGroup.srv"
  "srv/RemoveAllTargets.srv"
  "srv/RemoveAllClusters.srv"
  "msg/UpdateTargetCmdGroup.msg"
  "msg/UpdateClusterCmdGroup.msg"
  DEPENDENCIES std_msgs geometry_msgs LIBRARY_NAME ${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
