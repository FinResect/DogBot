import os
import yaml

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

    camera_params = os.path.join(
        get_package_share_directory('dogbot_core'),
        'config',
        'camera_params.yaml',
    )

    with open(camera_params) as f:
        _camera_cfg = yaml.safe_load(f)
        _rtsp_cfg = _camera_cfg.get('rtsp_stream_node', {}).get('ros__parameters', {})
        rtsp_enabled = _rtsp_cfg.get('enabled', False)

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
                plugin='dogbot_core::hardware::DogBot',
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

    load_camera = LoadComposableNodes(
        target_container='dogbot_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::camera::CameraNode',
                name='camera',
                parameters=[camera_params],
            ),
            ComposableNode(
                package='dogbot_core', 
                plugin='dogbot_core::vision::FollowingNode',
                name='following', 
                parameters=[camera_params],
            ),
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::vision::ColorDetectNode',
                name='color_detect',
                parameters=[camera_params],
            )
        ],
    )

    actions = [container, load_dogbot, load_camera]

    if rtsp_enabled:
        load_rtsp = LoadComposableNodes(
            target_container='dogbot_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='dogbot_core',
                    plugin='dogbot_core::camera::RtspStreamNode',
                    name='rtsp_stream',
                    parameters=[camera_params],
                ),
            ],
        )
        actions.append(load_rtsp)

    return LaunchDescription(actions)
