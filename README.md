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

- **Workspace**: A directory containing ROS2 packages, README etc.
- **Package**: A collection of nodes, libraries, and other resources.

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

### Building

ROS2 uses `colcon` for building packages.
```bash
colcon build
```
- `--symlink-install`: useful for development with python - no need to rebuild after edit.
- `--packages-select <package_name>`: build only the specified package.

**Important**: After building, source the setup file to make the packages available in the current terminal session:
```bash
source ~/.bashrc
```

### Running Nodes
To run a node, use the `ros2 run` command:
```bash
ros2 run <package_name> <node_executable>
```
Example:
```bash
ros2 run hello_world_py_pkg hello_world_node
```