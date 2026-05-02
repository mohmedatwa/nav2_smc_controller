# Nav2 Sliding Mode Controller (nav2_smc_controller)

A robust local trajectory planner (controller) plugin for the ROS 2 Navigation 2 (Nav2) stack. This package implements a Boundary-Layer Sliding Mode Control (SMC) law to track global paths, offering high precision and robustness against disturbances.

This controller uniquely supports both **Differential Drive** and **Omnidirectional (Holonomic/Mecanum)** kinematics out of the box.

## Features

* **Dual Kinematic Support:** Select between `diff` and `omni` drive types dynamically via parameters.
* **Chattering Suppression:** Utilizes a boundary-layer saturation function `sat(s / phi)` to smooth control outputs and prevent mechanical wear.
* **Derivative Kick Protection:** Smooth initialization upon receiving new paths to prevent sudden jerks in angular velocity.
* **Direct Lateral Control (Omni):** Fully utilizes the Y-axis (strafing) for holonomic bases to correct lateral errors without requiring unnecessary rotation.
* **Real-time Diagnostics:** Publishes target lookahead poses and internal sliding surface values for easy debugging via `rqt_plot`.

## Mathematical Overview

The controller calculates errors in the robot's local frame relative to a lookahead point on the global path. 

### Differential Drive (robot_type: "diff")
Because a differential drive robot cannot move laterally, lateral error (e_y) is coupled into the angular sliding surface to steer the robot back to the path.
* **Forward Surface:** s_v = e_x
* **Angular Surface:** s_w = e_y_dot + lambda * e_y

### Omnidirectional Drive (robot_type: "omni")
An omnidirectional robot can correct errors in all three degrees of freedom independently.
* **Forward Surface:** s_v = e_x
* **Lateral Surface:** s_lat = e_y
* **Heading Surface:** s_w = e_theta

The general control law applied to each surface is:
u = eta * s + k * sat(s / phi)

## Installation

Clone this repository into your ROS 2 workspace's `src` directory and build using `colcon`:

    cd ~/ros2_ws/src
    git clone https://github.com/mohmedatwa/nav2_smc_controller.git
    cd ~/ros2_ws
    colcon build --packages-select nav2_smc_controller
    source install/setup.bash

## Configuration

To use this controller in Nav2, configure your `controller_server` parameters to use `nav2_smc_controller/SMCController` under the `FollowPath` plugin.

### Example nav2_params.yaml

    controller_server:
      ros__parameters:
        controller_plugins: ["FollowPath"]
    
        FollowPath:
          plugin: "nav2_smc_controller/SMCController"
          robot_type: "omni"             # Options: "diff" or "omni"
          base_frame_id: "base_link"     # The local frame of your robot
          
          # Lookahead distance along the global path
          step_size: 0.2
    
          # Differential drive lateral coupling factor (Ignored in 'omni' mode)
          lambda: 1.0
    
          # Switching gains (k) - Determines aggressiveness toward the surface
          k_linear:  0.5
          k_lateral: 0.5   # Omni only
          k_angular: 1.0
    
          # Equivalent control gains (eta) - Keeps system on the surface
          eta_linear:  0.2
          eta_lateral: 0.2 # Omni only
          eta_angular: 0.5
    
          # Boundary layer thickness (phi) - Suppresses chattering
          boundary_layer: 0.1
    
          # Velocity Limits
          max_linear_velocity:  0.3     # m/s (vx)
          max_lateral_velocity: 0.3     # m/s (vy, omni only)
          max_angular_velocity: 1.0     # rad/s (wz)

## Parameters Description

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `robot_type` | string | `diff` | Kinematic mode: `"diff"` or `"omni"`. |
| `base_frame_id` | string | `base_link` | Robot's base coordinate frame. |
| `step_size` | double | `0.2` | Lookahead distance (meters) along the path to calculate error. |
| `lambda` | double | `1.0` | Tuning parameter coupling lateral position and velocity error (Diff only). |
| `k_*` | double | var. | Switching gains for linear/lateral/angular channels. Higher values close large errors faster. |
| `eta_*` | double | var. | Continuous control gains to push state toward s = 0. |
| `boundary_layer` | double | `0.1` | Boundary layer thickness (phi). Larger values reduce control chattering but may decrease tracking precision. |
| `max_*_velocity` | double | var. | Absolute clamping limits for the output velocity commands. |

## Published Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `smc/next_pose` | `geometry_msgs/PoseStamped` | The target lookahead point on the global path being tracked. |
| `smc/sliding_surfaces` | `std_msgs/Float64MultiArray` | The calculated sliding surface values. Array is `[s_v, s_w]` for diff, and `[s_v, s_lat, s_w]` for omni. Useful for parameter tuning via `rqt_plot`. |

## Author & License
* **Author:** Mohamed Atwa 
* **License:** Apache License 2.0
