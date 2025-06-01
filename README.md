# Learning ROS2

Install dependencies:
```bash
vcs import --recursive < px4.repos src
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:
```bash
colcon build --symlink-install
```

