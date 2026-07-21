from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = Node(
        package='rclcpp_components',
        executable='component_container',
        name='dogbot_container',
        output='screen',
    )

    load_dogbot = LoadComposableNodes(
        target_container='dogbot_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dogbot_core',
                plugin='DogBot',
                name='dogbot',
            ),
        ],
    )

    return LaunchDescription([container, load_dogbot])
