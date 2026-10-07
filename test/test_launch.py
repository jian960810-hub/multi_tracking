"""Load installed launch descriptions using the real ROS launch API."""
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import pytest
import yaml

pytest.importorskip('launch_ros')
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

ROOT = Path(__file__).resolve().parents[1]


def test_installed_launch_and_rviz():
    share = Path(get_package_share_directory('multi_tracking'))
    module = runpy.run_path(str(share / 'launch/hello.launch.py'))
    description = module['generate_launch_description']()
    assert isinstance(description, LaunchDescription)
    declared = {e.name for e in description.entities if isinstance(e, DeclareLaunchArgument)}
    assert {'scan_topic', 'start_lidar', 'rviz', 'record_scan', 'use_sim_time'} <= declared
    rviz = yaml.safe_load((share / 'rviz/multi_tracking_rviz_5_28.rviz').read_text())
    displays = rviz['Visualization Manager']['Displays']
    laser = next(d for d in displays if d['Class'].endswith('/LaserScan'))
    assert laser['Topic']['Reliability Policy'] == 'Best Effort'
    assert all(d['Class'].startswith('rviz_default_plugins/') for d in displays)
    assert (share / 'src/adaboost_trained_data_mess_430.txt').is_file()
    package = ET.parse(ROOT / 'package.xml').getroot()
    assert package.findtext('name') == 'multi_tracking'
    assert package.findtext('export/build_type') == 'ament_cmake'


def test_lidar_requires_explicit_supported_model():
    module = runpy.run_path(str(ROOT / 'launch/lidar.launch.py'))
    ctx = LaunchContext()
    ctx.launch_configurations.update(lds_model='', port='/dev/ttyUSB0', frame_id='base_scan')
    with pytest.raises(RuntimeError, match='lds_model'):
        module['launch_driver'](ctx)


@pytest.mark.parametrize('model,package,file', [
    ('LDS-01', 'hls_lfcd_lds_driver', 'hlds_laser.launch.py'),
    ('LDS-02', 'ld08_driver', 'ld08.launch.py'),
    ('LDS-03', 'coin_d4_driver', 'single_lidar_node.launch.py'),
])
def test_lidar_selection_without_hardware(model, package, file, tmp_path):
    # Driver path lookup is stubbed; launch objects are real. This does not
    # pretend to start a USB device or validate hardware compatibility.
    module = runpy.run_path(str(ROOT / 'launch/lidar.launch.py'))
    calls = []
    launch_dir = tmp_path / package / 'launch'
    launch_dir.mkdir(parents=True)
    (launch_dir / file).write_text('from launch import LaunchDescription\ndef generate_launch_description():\n    return LaunchDescription()\n')
    def lookup(name):
        calls.append(name)
        return str(tmp_path / name)
    module['launch_driver'].__globals__['get_package_share_directory'] = lookup
    ctx = LaunchContext()
    ctx.launch_configurations.update(lds_model=model, port='/dev/ttyUSB0', frame_id='base_scan')
    result = module['launch_driver'](ctx)
    assert calls == [package]
    assert isinstance(result[0], IncludeLaunchDescription)
    source = result[0].launch_description_source
    assert isinstance(source.get_launch_description(ctx), LaunchDescription)
    assert source.location.endswith(file)


def test_installed_headless_launch_starts_both_executables(tmp_path):
    import subprocess
    recording = tmp_path / 'launch_recording.txt'
    result = subprocess.run([
        'timeout', '--signal=INT', '5s', 'ros2', 'launch', 'multi_tracking',
        'hello.launch.py', 'rviz:=false', 'record_scan:=true',
        'output_file:=' + str(recording),
    ], capture_output=True, text=True, timeout=15)
    output = result.stdout + result.stderr
    assert result.returncode == 124, output  # stopped deliberately after startup
    assert 'Listening on /scan' in output, output
    assert 'Recording to' in output, output
    assert 'Traceback' not in output, output
    assert 'escalating' not in output, output
    assert output.count('process has finished cleanly') == 2, output
    assert recording.exists()
