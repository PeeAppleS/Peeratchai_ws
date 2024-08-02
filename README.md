# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

This is a ROS2 implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## How to test

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/params/vesc_config.yaml` to reflect any changes.
5. Build the packages `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
7. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`
# Peeratchai_ws

ros2 run slam_toolbox sync_slam_toolbox_node --ros-args --params-file /home/raspberry/Peeratchai_ws/mapper_params_online_sync_Peeratchai.yaml

ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link laser

ros2 launch sllidar_ros2 sllidar_s1_launch.py 

rviz2

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

ros2 run nav2_map_server map_saver_cli -f /home/raspberry/Peeratchai_ws/maps
//save map
