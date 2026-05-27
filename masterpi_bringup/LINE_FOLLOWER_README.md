# MasterPi ROS2 Yellow Line Follower

This patch adds a ROS2 camera-based yellow line follower for the Hiwonder MasterPi running Ubuntu Server + ROS2 Humble.

## What changed

- Added `masterpi_bringup/line_follower_node.py`
- Added `config/line_follower_params.yaml`
- Added `launch/line_follower.launch.py`
- Added console entry point `line_follower_node` in `setup.py`
- Changed `camera_node.publish_grayscale` to `false` in `config/robot_params.yaml` so yellow can be detected in color.

## Run

```bash
cd ~/ros2_ws_masterpi
cp -r /path/to/masterpi_bringup_line_follower ~/ros2_ws_masterpi/src/masterpi_bringup
# Or copy the changed files into your existing src/masterpi_bringup package.

colcon build --packages-select masterpi_bringup
source install/setup.bash
ros2 launch masterpi_bringup line_follower.launch.py
```

## Debug topics

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic hz /camera/image_raw
```

If you have a GUI machine in the same ROS_DOMAIN_ID, use:

```bash
ros2 run rqt_image_view rqt_image_view
```

Open:

- `/camera/image_raw`
- `/line_follower/mask`
- `/line_follower/debug_image`

## Tuning

Main file: `config/line_follower_params.yaml`

- If the yellow mask is weak, tune `h_min/h_max/s_min/v_min`.
- If the robot turns the wrong way, change `angular_sign` from `-1.0` to `1.0`.
- If it shakes too much, lower `kp_angular`.
- If it is too slow, increase `base_speed` gradually.
