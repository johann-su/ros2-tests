# Learning ROS2

[PX4 ros docs](https://docs.px4.io/main/en/ros2/)

## Setup
Setup the environment and verify it works.

Install dependencies:
```bash
vcs import --recursive < px4.repos src
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:
```bash
colcon build --symlink-install
```

Terminal 1:
Start the simulation (inside `/opt/PX4-Autopilot`) [more info](https://docs.px4.io/main/en/ros2/user_guide.html#start-the-client):
```bash
make px4_sitl gz_x500
```

Terminal 2:
Start XRCE-DDS agent (inside `/worksaces/ros2_ws`) [more info](https://docs.px4.io/main/en/ros2/user_guide.html#setup-the-agent):
```bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 3:
Listen to sensor data (inside `/worksaces/ros2_ws`) [more info](https://docs.px4.io/main/en/ros2/user_guide.html#running-the-example):
```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

## Controlling a vehicle
[Docs](https://docs.px4.io/main/en/ros2/user_guide.html#controlling-a-vehicle)