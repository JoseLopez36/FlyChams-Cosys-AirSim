cmake_minimum_required(VERSION 3.8)
project(airsim_wrapper)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(airsim_interfaces REQUIRED)

set(AIRSIM_ROOT $ENV{FLYCHAMS_AIRSIM_PATH})

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel")
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc
-fmax-errors=10")
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

set(INCLUDE_DIRS include 
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  )
include_directories(${INCLUDE_DIRS})

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser AirLib)

add_library(airsim_ros src/airsim_wrapper.cpp)
target_link_libraries(airsim_ros ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)
ament_target_dependencies(airsim_ros
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  airsim_interfaces
  std_msgs
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2
)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros AirLib)
ament_target_dependencies(airsim_node
  rclcpp
)

add_executable(airsim_get_image src/airsim_get_image.cpp)
target_link_libraries(airsim_get_image airsim_ros AirLib)
ament_target_dependencies(airsim_get_image
  rclcpp
)

add_executable(airsim_record_camera src/airsim_record_camera.cpp)
target_link_libraries(airsim_record_camera airsim_ros AirLib)
ament_target_dependencies(airsim_record_camera
  rclcpp
)

install(TARGETS airsim_node airsim_get_image airsim_record_camera
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS airsim_ros
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib)

install(
  DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(airsim_interfaces)

ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(airsim_settings_parser airsim_ros airsim_get_image airsim_record_camera)

ament_package()
