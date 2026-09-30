"""Use the same driver selection/arguments as ROBOTIS' Jazzy bringup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def launch_driver(context):
    model = LaunchConfiguration('lds_model').perform(context)
    drivers = {
        'LDS-01': ('hls_lfcd_lds_driver', 'hlds_laser.launch.py'),
        'LDS-02': ('ld08_driver', 'ld08.launch.py'),
        'LDS-03': ('coin_d4_driver', 'single_lidar_node.launch.py'),
    }
    if model not in drivers:
        raise RuntimeError('Set lds_model:=LDS-01, LDS-02 or LDS-03 to match your hardware; '
                           'or use start_lidar:=false with an existing /scan publisher.')
    package, launch_file = drivers[model]
    source = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)), launch_arguments={
            'port': LaunchConfiguration('port').perform(context),
            'frame_id': LaunchConfiguration('frame_id').perform(context),
            'namespace': '',
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lds_model', default_value=EnvironmentVariable('LDS_MODEL', default_value='')),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('frame_id', default_value='base_scan'),
        OpaqueFunction(function=launch_driver),
    ])
