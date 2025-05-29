# Learning ROS2

Official ROS2 jazzy documentation: https://docs.ros.org/en/jazzy/index.html <br>
Tutorial Series: https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&si=frFDZWzGmANyw5lF

## Basics

**ROS2 Versions:**
| Name | Release Date | EOL | LTS |
| --- | --- | --- | --- |
| Kilted Kaiju | May 23, 2025 | December 2026 | ❌ |
| Jazzy Jackal | May 23, 2024 | May 2029 | ✅ |
| Iron Irwini | May 23, 2023 | December 4, 2024 | ❌ |
| Humble Hawksbill | May 23, 2022 | May 2027 | ✅ |

---

**Vocabulary:**
- **Workspace**: A directory containing ROS2 packages, README etc.
- **Package**: A collection of nodes, libraries, and other resources. Can be external [ROS2 index](https://index.ros.org/packages/) or custom. See [Creating a Package](#creating-a-package).
    - **Build System**: `ament_cmake` for C++ packages, `ament_python` for Python packages.
    - **Dependencies**: Other packages that the package relies on, specified in `package.xml`.
    - **Build System**: `ament_cmake` for C++ packages, `ament_python` for Python packages.
    - **Dependencies**: Other packages that the package relies on, specified in `package.xml`.
- **Interface**: Type definition for messages, services, and actions. See [Interfaces](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html)

Example `srv/HelloWorld.srv`:
```plaintext
# request
string str
---
# response
string str
```

- **Node**: A process that performs computation. See [Understanding Nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- **Topic**: A named bus over which nodes exchange messages. See [Understanding Topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

Example: <br>
Terminal 1:
```bash
ros2 run demo_nodes_cpp talker
```
Terminal 2:
```bash
ros2 run demo_nodes_cpp listener
```

Graph: UI node introspection tool.
```bash
rqt_graph
```

- **service**: req/res communication model instead of pub/sub. Usefull for computations and performing actions on a node. See [Understanding Services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).
    - **service client**: a node that calls a service.
    - **service server**: a node that provides a service.
- **parameter**: a variable that can be set and retrieved by nodes, useful for configuration. Can only be modified at startup. Can be stored in YAML files. See [Understanding Parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

Running service with parameter yaml file:
```bash
ros2 run hello_world_py_pkg simple_params --ros-args --params-file ./src/hello_world_py_pkg/config/params.yaml
```

- **action**: long running task that can be preempted or cancelled. See [Understanding Actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).

![ROS2 Action](https://docs.ros.org/en/jazzy/_images/Action-SingleActionClient.gif)

### ROS2 CLI Tools

`ros2 -h` to see the list of available commands. <br>

**Common Commands:**
- `ros2 run <package_name> <node_executable>`: Run a node from a package.

Node:
- `ros2 node list`: List all running nodes.
- `ros2 node info <node_name>`: Get information about a specific node.

Topics:
- `ros2 topic list`: List all active topics.
- `ros2 topic echo <topic_name>`: Print messages from a topic to the console.

Services:
- `ros2 service list`: List all available services.

Actions:
- `ros2 action list`: List all active actions.

Packages:
- `ros2 pkg create <package_name>`: Create a new package ([more info](#creating-a-package)).
- `ros2 pkg list`: List all installed packages.
- `ros2 pkg executables <package_name>`: List all executables in a package.

Interfaces:
- `ros2 interface list`: List all available interfaces (messages, services, actions).
- `ros2 interface show <interface_name>`: Show the definition of a specific interface.

rqt:
- `rqt`: Launch the rqt GUI for introspection and visualization.
- `rqt_graph`: Launch the rqt graph tool to visualize the node and topic connections.
- `rqt_console`: Launch the rqt console for viewing log messages.

### Creating a Package

To create a new package, use the following command:
```bash
ros2 pkg create <package_name> \
    --build-type <build_system> \
    --dependencies <dependencies> \
    --destination-directory ./src
```
- `<package_name>`: commonly robot_name_controller, robot_name_driver, robot_name_simulation, etc.
- `<build_system>`: `ament_cmake` for c++ packages, `ament_python` for Python packages. (ament is the default build system for ROS2 used by colcon).
- `<dependencies>`: format "dependency1 dependency2 ..."
    - `rclcpp` ros2 C++ client library
    - `rclpy` ros2 python client library

**Important**: `colcon build` and `source ~/.bashrc` after creating a new package to use it.

-> See src/hello_world_py_pkg for an example of a Python package. Modify the `setup.py` file to include dependencies and entry points.

Note: It is possible to create both C++ and Python packages in the same workspace.

Template for a python node:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class CustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        self.get_logger().info('Node has been started!')

def main(args=None):
    rclpy.init(args=args)

    node = CustomNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Template for a C++ node:
```cpp
#include "rclcpp/rclcpp.hpp"

class CustomNode : public rclcpp::Node
{
private:
public:
    CustomNode() : Node("custom_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node has been started!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CustomNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
```

### Building

ROS2 uses `colcon` for building packages ([more info](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)):
```bash
colcon build
```
- `--symlink-install`: useful for development with python - no need to rebuild after edit.
- `--packages-select <package_name>`: build only the specified package.

**Important**: After building, source the setup file to make the packages available in the current terminal session:
```bash
source ~/.bashrc
```

### Running/Launching Nodes

To run a node, use the `ros2 run` command ([more info](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)):
```bash
ros2 run <package_name> <node_executable>
```
Example:
```bash
ros2 run hello_world_py_pkg hello_world_node
```

#### Launch files

Launch multiple nodes with specified parameters ([more info](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)). <br>
Either launch directory in the package or a seperate bringup package.

Launch file template (end in `.launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Define nodes to launch

    return ld
```
Example:
```bash
ros2 launch hello_world_py_pkg hello_world.launch.py
```

**Note**: Dont forget to add the dependencies in `package.xml`.

### Dependency Management with `rosdep`

- `rosdep update`: Update the rosdep database.
- `rosdep install --from-paths src -y --ignore-src`: Install dependencies for the current workspace.
    - `--from-paths src`: Specify the path to check for `package.xml`.
    - `-y`: Automatically answer yes to prompts.
    - `--ignore-src`: Ignore source packages, only install dependencies.

Add `<depend>` tags in `package.xml` to specify dependencies for your package. Example:
```xml
<package format="3">
    ...
    <depend>package-key</depend>
    ...
</package>
```
`package-key` can be:
- name of the package if it is a native ROS2 package **and** released into the ROS2 [index](https://github.com/ros/rosdistro) at `jazzy/distribution.yaml`.
- if the package is not ROS2 native, it can be a system dependency (e.g. `libboost-dev`) or a Python package (e.g. `numpy`).
    - [`rosdep/base.yaml`](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml) contains the apt system dependencies
    - [`rosdep/python.yaml`](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml)  contains the Python dependencies
The key is the name of the package from these yaml files.

## Ros Packages

### Turtlesim
A simple simulator for testing stuff.

List all executables in the package:
```bash
ros2 pkg executables turtlesim
```

Run a single node:
```bash	
ros2 run turtlesim turtlesim_node
```

Add keyboard control to the turtle:
```bash
ros2 run turtlesim turtle_teleop_key
```

### tf2
Package for broadcasting transformations ([more info](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)).

Types of transformations:
- **static**: Fixed transformations that do not change over time.
- **dynamic**: Transformations that can change over time, such as moving robots or sensors.

### URDF

Unified Robot Description Format (URDF) - describe a robot and use it for simulation, visualization and kinematics ([more info](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)). 

**Terminology:**
- **Link**: A rigid body in the robot model. The following characteristics can be defined:
    - **visual**: The visual representation of the link (e.g. mesh, geometry).
    - **collision**: The collision representation of the link (e.g. mesh, geometry).
    - **inertial**: The inertial properties of the link (e.g. mass, inertia matrix).
- **Joint**: A connection between two links that allows relative motion with different DoF:
    - **Revolut**: Discrete rotation around an axis (rotation with start and stop angle, e.g. robotic arm).
    - **Continuous**: Continuous rotation around an axis (e.g. wheels).
    - **Prismatic**: Linear translation along an axis (e.g. linear actuator).
    - **Fixed**: No relative motion between links (e.g. base of a robot).

**Note**: One link alsways has exactly one parent (except for the first one) but can have many children (connected through joins).

Visualize the joint hierarchy:
```bash	
ros2 run tf2_tools view_frames
```

Template for a URDF file `example.urdf.xacro`:
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
    <link name="link_name">
        <visual>
            <geometry>
                <!-- Geometry of the link, e.g. box, cylinder, mesh -->
            </geometry>
            <origin xyz="x y z" rpy="roll pitch yaw"/>
            <material name="material_name">
                <!-- Material properties, e.g. color -->
            </material>
        </visual>
        <collision>
            <geometry>
                
            </geometry>
            <origin xyz="x y z" rpy="roll pitch yaw"/>
        </collision>
        <inertial>
            <mass value="mass_value"/>
            <inertia ixx="ixx_value" ixy="ixy_value" ixz="ixz_value" iyy="iyy_value" iyz="iyz_value" izz="izz_value"/>
            <origin xyz="x y z" rpy="roll pitch yaw"/>
        </inertial>
    </link>
    <joint name="joint_name" type="joint_type">
        <parent link="parent_link_name"/>
        <child link="child_link_name"/>
        <!-- Joint properties -->
    </joint>
    <!-- Additional links and joints -->
</robot>
```
**Note**: Use `xacro` to process the URDF file, which allows for macros and parameters. To convert a `.xacro` file to a `.urdf` file, use:
```bash
ros2 run xacro xacro example.urdf.xacro -o example.urdf
```

### RViz

launch RViz:
```bash
rviz2
```

### Gazebo
