# AirSim Wrapper: ROS2 Interface for AirSim Simulator

A ROS2 wrapper for the AirSim simulator, providing a bridge between ROS2 and AirSim's C++ client library.

## Overview

The `airsim_wrapper` package provides a complete ROS2 interface to the AirSim simulator for the Flying Chameleons (FlyChams) project. It handles:

1. **Vehicle Communication** - Exposes AirSim's vehicle control APIs as ROS2 topics and services
2. **Sensor Data** - Publishes sensor data from AirSim as ROS2 topics
3. **Transformation Management** - Handles TF2 frames for vehicles and sensors
4. **Simulation Control** - Provides services for controlling the simulation (pause/resume/reset)
5. **GUI Interaction** - Allows manipulation of AirSim's window displays

## Nodes

### AirSim Wrapper Node

The main node (`airsim_node`) that establishes a connection to the AirSim simulator and provides ROS2 interfaces:

- Initializes communication with AirSim using RPC
- Creates publishers, subscribers, and services for each vehicle and camera
- Maintains state synchronization between ROS2 and AirSim
- Handles transformation broadcasts for vehicles and cameras

### ROS2 Interfaces

#### Published Topics

- `/airsim/[vehicle_name]/global/state/odom` - Global vehicle odometry
- `/airsim/[vehicle_name]/local/state/odom` - Local vehicle odometry
- `/clock` - Simulation clock

#### Subscribed Topics

- `/airsim/[vehicle_name]/local/cmd/velocity` - Local velocity commands
- `/airsim/[vehicle_name]/local/cmd/position` - Local position commands
- `/airsim/[vehicle_name]/global/cmd/position` - Global position commands
- `/airsim/[vehicle_name]/gimbals/cmd/orientation` - Gimbal angle commands
- `/airsim/[vehicle_name]/cameras/cmd/fov` - Camera FOV commands
- `/airsim/windows/cmd/image` - Window image display commands
- `/airsim/windows/cmd/rectangle` - Window rectangle drawing commands
- `/airsim/windows/cmd/string` - Window text display commands
- `/airsim/targets/cmd/update` - Target update commands
- `/airsim/clusters/cmd/update` - Cluster update commands

#### Services

- `/airsim/load_level` - Load a new level/environment
- `/airsim/reset` - Reset the simulation
- `/airsim/run` - Start/continue the simulation
- `/airsim/pause` - Pause the simulation
- `/airsim/vehicles/cmd/enable_control` - Enable API control of vehicle
- `/airsim/vehicles/cmd/arm_disarm` - Arm or disarm vehicle
- `/airsim/vehicles/cmd/takeoff` - Command vehicle takeoff
- `/airsim/vehicles/cmd/land` - Command vehicle landing
- `/airsim/vehicles/cmd/hover` - Command vehicle to hover
- `/airsim/targets/cmd/add` - Add a group of targets to simulation
- `/airsim/clusters/cmd/add` - Add a group of clusters to simulation
- `/airsim/targets/cmd/remove_all` - Remove all targets from simulation
- `/airsim/clusters/cmd/remove_all` - Remove all clusters from simulation