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

    camera_top_params = os.path.join(
        get_package_share_directory('dogbot_core'),
        'config',
        'camera_top_params.yaml',
    )

    rtsp_stream_params = os.path.join(
        get_package_share_directory('dogbot_core'),
        'config',
        'rtsp_stream_params.yaml',
    )

    following_params = os.path.join(
        get_package_share_directory('dogbot_core'),
        'config',
        'following_params.yaml',
    )

    color_detect_params = os.path.join(
        get_package_share_directory('dogbot_core'),
        'config',
        'color_detect_params.yaml',
    )

    color_detect_nonblue_params = os.path.join(
        get_package_share_directory('dogbot_core'),
        'config',
        'color_detect_nonblue_params.yaml',
    )

    with open(rtsp_stream_params) as f:
        _rtsp_cfg = yaml.safe_load(f).get('rtsp_stream', {}).get('ros__parameters', {})
        rtsp_enabled = _rtsp_cfg.get('enabled', False)

    container = Node(
        package='rclcpp_components',
        executable='component_container',
        name='dogbot_container',
        output='screen',
        # prefix='taskset -c 0,1',
    )

    vision_container = Node(
        package='rclcpp_components',
        executable='component_container_mt',
        name='vision_container',
        output='screen',
        # prefix='taskset -c 2,3',
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
        target_container='vision_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::camera::CameraNode',
                name='camera',
                parameters=[camera_params],
            ),
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::camera::CameraTopNode',
                name='camera_top',
                parameters=[camera_top_params],
            ),
            ComposableNode(
                package='dogbot_core', 
                plugin='dogbot_core::vision::FollowingNode',
                name='following', 
                parameters=[following_params],
            ),
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::vision::ColorDetectNode',
                name='color_detect',
                parameters=[color_detect_params],
            ),
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::vision::ColorDetectNonblueNode',
                name='color_detect_nonblue',
                parameters=[color_detect_nonblue_params],
            ),
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::vision::ColorListenerNode',
                name='color_listener',
            ),
        ],
    )

    actions = [container, vision_container, load_dogbot, load_camera]

    if rtsp_enabled:
        load_rtsp = LoadComposableNodes(
            target_container='vision_container',
            composable_node_descriptions=[
            ComposableNode(
                package='dogbot_core',
                plugin='dogbot_core::camera::RtspStreamNode',
                name='rtsp_stream',
                parameters=[rtsp_stream_params],
            ),
            ],
        )
        actions.append(load_rtsp)

    return LaunchDescription(actions)
