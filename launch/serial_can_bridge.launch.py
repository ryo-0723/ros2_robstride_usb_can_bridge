"""@brief YAMLとlaunch引数でシリアルブリッジを起動する."""

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
    """@brief YAMLを読み込み、指定されたlaunch引数で上書きする.

    @param context launch引数を評価する実行コンテキスト
    @return シリアルブリッジの起動アクションの一覧
    @throws RuntimeError 指定したYAMLファイルが存在しない場合
    """
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
    """@brief 任意の設定をlaunch引数として宣言する.

    @return ノード起動と引数宣言をまとめたLaunchDescription
    @note 未指定のパラメータはノードの既定値を使う
    """
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
