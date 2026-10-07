"""Start tracking and RViz; optional on-device LiDAR and scan recorder."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('multi_tracking')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('marker_topic', default_value='/visualization_marker'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('fixed_frame', default_value='base_scan'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([share, 'resource', 'tracking.yaml'])),
        DeclareLaunchArgument('model_path', default_value=PathJoinSubstitution([share, 'resource', 'adaboost_trained_data_mess_430.txt'])),
        DeclareLaunchArgument('start_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('lds_model', default_value=EnvironmentVariable('LDS_MODEL', default_value='')),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('record_scan', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('output_file', default_value=''),
        DeclareLaunchArgument('duration_seconds', default_value='360.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'lidar.launch.py'])),
            condition=IfCondition(arg('start_lidar')),
            launch_arguments={'lds_model': arg('lds_model'), 'port': arg('lidar_port'),
                              'frame_id': arg('fixed_frame')}.items()),
        Node(package='multi_tracking', executable='real_time_6_5', name='leg_tracker',
             output='screen', parameters=[arg('params_file'), {
                 'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool),
                 'scan_topic': ParameterValue(arg('scan_topic'), value_type=str),
                 'marker_topic': ParameterValue(arg('marker_topic'), value_type=str),
                 'model_path': ParameterValue(arg('model_path'), value_type=str)}]),
        Node(package='multi_tracking', executable='getdata', name='scan_recorder',
             condition=IfCondition(arg('record_scan')), output='screen', parameters=[{
                 'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool),
                 'scan_topic': ParameterValue(arg('scan_topic'), value_type=str),
                 'output_file': ParameterValue(arg('output_file'), value_type=str),
                 'duration_seconds': ParameterValue(arg('duration_seconds'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(arg('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'multi_tracking_rviz_5_28.rviz']),
                        '-f', arg('fixed_frame')],
             remappings=[('/scan', arg('scan_topic')), ('/visualization_marker', arg('marker_topic'))],
             parameters=[{'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}]),
    ])
