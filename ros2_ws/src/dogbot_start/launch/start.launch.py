import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

PACKAGES = (
    'dogbot_core',
    'serial_bridge',
)

DEFAULTS = {
    'output': 'screen',
}


def _load_config(package_name):
    share = get_package_share_directory(package_name)
    path = os.path.join(share, 'config', 'nodes.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    actions = []
    for pkg in PACKAGES:
        config = _load_config(pkg)
        for entry in config.get('nodes', []):
            cfg = {**DEFAULTS, **entry}
            actions.append(Node(package=pkg, **cfg))
    return LaunchDescription(actions)


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
