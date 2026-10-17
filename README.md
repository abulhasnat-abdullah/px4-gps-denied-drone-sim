# px4-gps-denied-drone-sim

ROS 2 + PX4 SITL workspace for flying a quadrotor indoors without GPS. A 2D LiDAR gives the drone a position estimate that doesn't drift or reset, and that estimate is fed back into PX4's EKF2 as external vision. This is the positioning chain behind [VENTRA](https://github.com/abulhasnat-abdullah/VENTRA).

## The problem

With only optical flow for horizontal aiding, EKF2's position kept resetting towards (0, 0) whenever flow quality dropped (low light, low texture, out of range). This workspace adds a LiDAR-anchored estimate so flow is no longer the only source.

## How it works

```
LiDAR /scan
   │
   ▼
rf2o_laser_odometry      frame-to-frame scan matching   → odom → base_link
   │
   ▼
slam_toolbox (async)     scan matching against a map,   → map → odom
   │                     loop closure corrects drift
   ▼
vision_odom_bridge       map → base_link, converted ENU/FLU → FRD
   │
   ▼
/fmu/in/vehicle_visual_odometry  →  EKF2 fuses it (EKF2_EV_CTRL)
```

- `odom_bridge_node` turns PX4's `/fmu/out/vehicle_odometry` (NED/FRD) into `nav_msgs/Odometry` (ENU) and publishes the static sensor transforms, so RViz2 lines up. It also starts the Gazebo clock and LiDAR bridges.
- `vision_odom_bridge` reads the SLAM pose through tf2 and publishes it as `px4_msgs/VehicleOdometry` in the FRD pose frame (locked to the start heading, since the SLAM map has no North reference).

See [`src/odom_bridge/README.md`](src/odom_bridge/README.md) for the full setup notes and EKF2 parameters.

## Workspace

| Package | Purpose |
|---|---|
| `odom_bridge` | PX4 ↔ ROS 2 odometry bridges, SLAM Toolbox config, full-stack launch |
| `px4_msgs`, `px4_ros_com` | PX4's ROS 2 message definitions and helpers (upstream) |
| `rf2o_laser_odometry` | LiDAR scan-matching odometry (upstream, MAPIRlab) |
| `ROS2-PX4_Drone_Teleoperation_Using_Joystick` | offboard teleoperation (`px4_offboard`, `px4_bringup`) |

`scripts/visualize.py` and `scripts/real_visualize.py` plot the estimate during and after a flight.

## Run

Requires PX4-Autopilot SITL with Gazebo, ROS 2 Jazzy, and the Micro XRCE-DDS agent.

```bash
cd ~/px4_ros2_ws
colcon build
source install/setup.bash

# PX4 SITL + Gazebo and the XRCE-DDS agent running, then:
./run_cmds/slam.sh
```

`slam.sh` starts the odometry bridge, rf2o, slam_toolbox, the vision bridge and keyboard teleop in order. Edit the `slam_params_file` path in it to match your workspace.
