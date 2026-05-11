from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
    
def generate_launch_description():
    # Declare launch arguments
    host_ip_arg = DeclareLaunchArgument('host_ip', default_value='localhost')
    host_port_arg = DeclareLaunchArgument('host_port', default_value='41451')
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='0')
    image_path_arg = DeclareLaunchArgument('image_path', default_value=os.environ.get('FLYCHAMS_ROS2_PATH') + '/experiments/images/image.png')

    # Create LaunchConfiguration objects to use the arguments
    host_ip = LaunchConfiguration('host_ip')
    host_port = LaunchConfiguration('host_port')
    camera_name = LaunchConfiguration('camera_name')
    image_path = LaunchConfiguration('image_path')

    # Generate launch description
    ld = []

    # Add the argument declarations to the launch description
    ld.append(host_ip_arg)
    ld.append(host_port_arg)
    ld.append(camera_name_arg)
    ld.append(image_path_arg)

    # Add AirSim Get Image node
    ld.append(
        Node(
            package='airsim_wrapper',
            executable='airsim_get_image',
            name='airsim_get_image',
            output='screen',
            namespace='airsim',
            parameters=[{
                'host_ip': host_ip,
                'host_port': host_port,
                'camera_name': camera_name,
                'image_path': image_path
            }]
        )
    )

    return LaunchDescription(ld)