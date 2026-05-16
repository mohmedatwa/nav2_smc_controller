# nav2_smc_controller

A Sliding Mode Controller (SMC) plugin for the [ROS 2 Nav2](https://nav2.ros.org) stack. Supports both **differential drive** and **omnidirectional (mecanum)** robots.

---

## How It Works

The controller computes velocity commands by calculating the error between the robot and a lookahead point on the global path, then applying an SMC law with boundary-layer saturation to suppress chattering.

**Control law applied to each channel:**
```
u = η·s + k·sat(s / φ)
```

### Differential Drive
| Surface | Formula |
|:--------|:--------|
| Forward | `s_v = e_x` |
| Angular | `s_w = ė_y + λ·e_y` |

### Omnidirectional
| Surface | Formula |
|:--------|:--------|
| Forward | `s_v = e_x` |
| Lateral | `s_lat = e_y` |
| Heading | `s_w = e_θ` |

---

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/mohmedatwa/nav2_smc_controller.git
cd ~/ros2_ws
colcon build --packages-select nav2_smc_controller
source install/setup.bash
```

---

## Configuration

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_smc_controller/SMCController"

      robot_type: "omni"        # "diff" or "omni"
      base_frame_id: "base_link"
      step_size: 0.2            # lookahead distance (m)
      lambda: 1.0               # diff only: lateral coupling factor

      k_linear:  0.5            # switching gains
      k_lateral: 0.5            # omni only
      k_angular: 1.0

      eta_linear:  0.2          # equivalent control gains
      eta_lateral: 0.2          # omni only
      eta_angular: 0.5

      boundary_layer: 0.1       # chattering suppression (φ)

      max_linear_velocity:  0.3 # m/s
      max_lateral_velocity: 0.3 # m/s, omni only
      max_angular_velocity: 1.0 # rad/s
```

---

## Parameters

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `robot_type` | `diff` | Kinematic mode: `diff` or `omni` |
| `base_frame_id` | `base_link` | Robot base frame |
| `step_size` | `0.2` | Lookahead distance along the path (m) |
| `lambda` | `1.0` | Lateral-to-angular coupling. Diff only |
| `k_linear / k_lateral / k_angular` | `0.5 / 0.5 / 1.0` | Switching gains — aggressiveness toward surface |
| `eta_linear / eta_lateral / eta_angular` | `0.2 / 0.2 / 0.5` | Equivalent control gains — tracking on surface |
| `boundary_layer` | `0.1` | Boundary layer φ — larger reduces chattering |
| `max_linear_velocity` | `0.3` | Max vx (m/s) |
| `max_lateral_velocity` | `0.3` | Max vy (m/s). Omni only |
| `max_angular_velocity` | `1.0` | Max wz (rad/s) |

---

## Topics

| Topic | Type | Description |
|:------|:-----|:------------|
| `smc/next_pose` | `geometry_msgs/PoseStamped` | Current lookahead target on the path |
| `smc/sliding_surfaces` | `std_msgs/Float64MultiArray` | Surface values `[s_v, s_w]` or `[s_v, s_lat, s_w]` |

Use `rqt_plot /smc/sliding_surfaces/data[0]` to monitor surfaces during tuning.

---

## Tuning

1. Start with `k_linear: 0.3`, `k_angular: 0.5`, all `eta_*` at `0.1`
2. If oscillating → increase `boundary_layer`
3. If tracking is sluggish → increase `eta_*`
4. If response to large errors is slow → increase `k_*`
5. For omni robots, tune linear and lateral channels before angular

---

## Testing

```bash
colcon test --packages-select nav2_smc_controller
colcon test-result --verbose
```

---

## License

Apache License 2.0 — © Mohamed Atwa
