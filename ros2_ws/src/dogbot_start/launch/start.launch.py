import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    leg_controller_params = os.path.join(
        get_package_share_directory('dogbot_core'),
        'config',
        'leg_controller_params.yaml',
    )

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
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::controller::LegController',
                name='leg_controller',
                parameters=[leg_controller_params],
            ),
        ],
    )

    return LaunchDescription([container, load_dogbot])
