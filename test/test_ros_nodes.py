"""Real rclpy/DDS integration tests; skipped only on non-ROS local machines."""
import io
import time
import uuid

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from multi_tracking.real_time_6_5 import LegTrackerNode
from multi_tracking.getdata import ScanRecorder


@pytest.fixture
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def sample_scan(seq=1, empty=False, frame='base_scan'):
    scan = LaserScan()
    scan.header.frame_id = frame
    scan.header.stamp.sec = 100 + seq // 5
    scan.header.stamp.nanosec = (seq % 5) * 200000000
    scan.angle_min = 0.0
    scan.angle_increment = .01
    scan.angle_max = .99
    scan.range_min = .1
    scan.range_max = 10.0
    ranges = [float('inf')] * 100
    if not empty:
        ranges[20:26] = [1.] * 6
        ranges[40:46] = [1.] * 6
    scan.ranges = ranges
    return scan


def make_tracker(tmp_path, scan_topic, marker_topic):
    # A deterministic 1-stump fixture classifies every >=3 point cluster as a leg.
    # The real trained model is independently checked/replayed in test_tracking_core.
    model = tmp_path / 'model.txt'
    np.savetxt(model, np.array([[1.], [0.], [2.], [1.], [1.]]))
    return LegTrackerNode(parameter_overrides=[
        Parameter('scan_topic', value=scan_topic),
        Parameter('marker_topic', value=marker_topic),
        Parameter('model_path', value=str(model)),
    ])


def test_best_effort_laser_reaches_tracker_and_publishes_markers(ros, tmp_path):
    prefix = '/test_' + uuid.uuid4().hex
    tracker = make_tracker(tmp_path, prefix + '/scan', prefix + '/markers')
    observer = Node('tracker_test_' + uuid.uuid4().hex)
    publisher = observer.create_publisher(LaserScan, prefix + '/scan', qos_profile_sensor_data)
    markers = []
    subscription = observer.create_subscription(Marker, prefix + '/markers', markers.append, 100)
    executor = SingleThreadedExecutor()
    executor.add_node(tracker)
    executor.add_node(observer)
    try:
        # First establish DDS discovery, then send distinct stamped scans.
        deadline = time.monotonic() + 10
        while (publisher.get_subscription_count() == 0 or subscription.get_publisher_count() == 0):
            assert time.monotonic() < deadline, 'DDS endpoints did not discover each other'
            executor.spin_once(timeout_sec=.05)
        seq = 1
        while not any(m.type == Marker.CYLINDER and m.action == Marker.ADD for m in markers):
            assert time.monotonic() < deadline, 'No confirmed track from best-effort LaserScan'
            publisher.publish(sample_scan(seq))
            seq += 1
            for _ in range(4):
                executor.spin_once(timeout_sec=.02)
        added = [m for m in markers if m.action == Marker.ADD]
        assert len({m.id for m in added}) == 1
        assert all(m.header.frame_id == 'base_scan' for m in added)
        assert all(np.isfinite([m.pose.position.x, m.pose.position.y]).all() for m in added)
        while not any(m.action == Marker.DELETE for m in markers):
            assert time.monotonic() < deadline, 'Expired track was not deleted'
            publisher.publish(sample_scan(seq, empty=True))
            seq += 1
            for _ in range(4):
                executor.spin_once(timeout_sec=.02)
        assert not tracker.tracker.tracks
    finally:
        executor.shutdown()
        tracker.destroy_node()
        observer.destroy_node()


def test_scan_time_frame_reset_and_invalid_metadata(ros, tmp_path):
    node = make_tracker(tmp_path, '/direct_scan', '/direct_markers')
    try:
        node.on_scan(sample_scan(1))
        first = node.tracker.tracks[0].id
        node.on_scan(sample_scan(1))
        assert node.tracker.tracks[0].pair_hits == 1  # duplicate stamp ignored
        node.on_scan(sample_scan(2, frame='other_laser'))
        assert node.tracker.tracks[0].id != first
        node.on_scan(sample_scan(1, frame='other_laser'))  # backwards clock
        assert len(node.tracker.tracks) == 1
        old_time = node.last_stamp
        invalid = sample_scan(3)
        invalid.angle_increment = 0.0
        node.on_scan(invalid)
        assert node.last_stamp == old_time
        invalid = sample_scan(3, frame='')
        node.on_scan(invalid)
        assert node.last_stamp == old_time
        node.on_scan(sample_scan(100))  # large gap resets stale state
        assert len(node.tracker.tracks) == 1
        zero_stamp = sample_scan()
        zero_stamp.header.stamp.sec = 0
        zero_stamp.header.stamp.nanosec = 0
        node.on_scan(zero_stamp)
        assert node.last_time_source == 'receipt'
    finally:
        node.destroy_node()


def test_recorder_flushes_variable_count_and_retains_invalid_returns(ros, tmp_path):
    path = tmp_path / 'recording.txt'
    node = ScanRecorder(parameter_overrides=[Parameter('output_file', value=str(path)),
                                            Parameter('duration_seconds', value=1.)])
    try:
        scan = sample_scan()
        scan.ranges = [1., float('inf'), 0., float('nan'), 2.]
        node.on_scan(scan)
        text = path.read_text()
        assert 'count=5' in text
        data = np.loadtxt(io.StringIO(text))
        assert data.shape == (5, 2)
        assert np.isinf(data[1, 1]) and np.isnan(data[3, 1])
        np.testing.assert_allclose(data[:, 0], np.arange(5) * scan.angle_increment)
        node.started_at -= 2.
        node.check_deadline()
        assert node.finished and node.stream.closed
        node.on_scan(sample_scan(2))
        assert node.scan_count == 1
    finally:
        node.destroy_node()


def test_recorder_best_effort_subscription(ros, tmp_path):
    topic = '/record_' + uuid.uuid4().hex
    path = tmp_path / 'dds.txt'
    node = ScanRecorder(parameter_overrides=[Parameter('scan_topic', value=topic),
                                            Parameter('output_file', value=str(path)),
                                            Parameter('duration_seconds', value=0.)])
    sender = Node('recorder_test_' + uuid.uuid4().hex)
    pub = sender.create_publisher(LaserScan, topic, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sender)
    try:
        deadline = time.monotonic() + 10
        while node.scan_count == 0:
            assert time.monotonic() < deadline
            pub.publish(sample_scan())
            executor.spin_once(timeout_sec=.05)
        assert '# scan' in path.read_text()
    finally:
        executor.shutdown()
        node.destroy_node()
        sender.destroy_node()
    assert path.read_text().endswith('\n')
