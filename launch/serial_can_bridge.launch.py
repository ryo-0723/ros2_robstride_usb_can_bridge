"""Launch the serial bridge with optional YAML and explicit parameter overrides."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PARAMETER_TYPES = {
    'device': str,
    'baud_rate': int,
    'tx_topic': str,
    'rx_topic': str,
    'frame_id': str,
    'poll_period': float,
    'write_timeout': float,
}


def launch_node(context):
    """Apply a YAML file first, then only the launch arguments that were supplied."""
    parameters = []
    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file:
        if not Path(params_file).is_file():
            raise RuntimeError(f'Parameter file does not exist: {params_file}')
        parameters.append(params_file)

    overrides = {}
    for name, value_type in PARAMETER_TYPES.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value_type(value)
    if overrides:
        parameters.append(overrides)

    return [Node(
        package='serial_can_bridge',
        executable='serial_can_bridge',
        name='serial_can_bridge',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    """Declare optional settings while keeping defaults in the node."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional YAML file; omitted parameters use node defaults.'),
        *[DeclareLaunchArgument(
            name, default_value='',
            description='Optional override of YAML or node default.')
          for name in PARAMETER_TYPES],
        OpaqueFunction(function=launch_node),
    ])
