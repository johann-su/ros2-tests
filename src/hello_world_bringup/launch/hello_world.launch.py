from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    # Very simple
    py_node = Node(
        package='hello_world_py_pkg',
        executable='hello_world_node',
    )
    ld.add_action(py_node)

    cpp_node = Node(
        package='hello_world_cpp_pkg',
        executable='hello_world_node',
    )
    ld.add_action(cpp_node)

    # More options available
    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="my_talker", # Optional: specify a custom name for the node
        remappings=[
            ('chatter', 'my_chatter')  # Optional: remap topics
        ],
        parameters=[{'param1': 'value1', 'param2': 42}],  # Optional: set parameters
    )
    ld.add_action(talker_node)

    listener_node = Node(
        package="demo_nodes_cpp",
        executable="listener",
        name="my_listener",
        remappings=[
            ('chatter', 'my_chatter')  # Optional: remap topics
        ],
    )
    ld.add_action(listener_node)

    return ld