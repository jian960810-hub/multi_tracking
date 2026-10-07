#!/usr/bin/env python3
"""ROS 2 Jazzy LaserScan -> AdaBoost leg detector -> multi-person tracker."""
import colorsys
import math
import signal
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from tracking_core import AdaBoostModel, MultiTracker, scan_segments


class LegTrackerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('leg_tracker', **kwargs)
        default_model = str(Path(get_package_share_directory('multi_tracking')) /
                            'src' / 'adaboost_trained_data_mess_430.txt')
        defaults = {
            'scan_topic': '/scan', 'marker_topic': '/visualization_marker',
            'model_path': default_model, 'segment_distance': 0.1,
            'min_segment_points': 3, 'pair_distance': 0.8,
            'association_gate': 250.0, 'default_dt': 0.2,
            'reset_gap': 1.0, 'marker_lifetime': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.settings = {k: self.get_parameter(k).value for k in defaults}
        for name in ('segment_distance', 'pair_distance', 'association_gate',
                     'default_dt', 'reset_gap', 'marker_lifetime'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.settings['min_segment_points'] < 3:
            raise ValueError('min_segment_points must be >= 3')
        self.model = AdaBoostModel(self.settings['model_path'])
        self.tracker = MultiTracker(self.settings['pair_distance'], self.settings['association_gate'])
        self.publisher = self.create_publisher(Marker, self.settings['marker_topic'], 100)
        self.subscription = self.create_subscription(
            LaserScan, self.settings['scan_topic'], self.on_scan, qos_profile_sensor_data)
        self.last_stamp = None
        self.last_frame = None
        self.last_time_source = None
        self.visible_ids = set()
        self.get_logger().info(f"Listening on {self.settings['scan_topic']}; model: {self.settings['model_path']}")

    def on_scan(self, scan):
        if not scan.header.frame_id:
            self.get_logger().warning('Ignoring scan with empty frame_id', throttle_duration_sec=5.0)
            return
        try:
            segments = scan_segments(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, self.settings['segment_distance'],
                self.settings['min_segment_points'])
            legs = self.model.detect(segments)
        except (ValueError, ArithmeticError) as exc:
            self.get_logger().warning(f'Ignoring invalid scan: {exc}', throttle_duration_sec=5.0)
            return
        stamp = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        source = 'message' if stamp else 'receipt'
        if not stamp:
            stamp = self.get_clock().now().nanoseconds
        dt = self.settings['default_dt']
        changed = self.last_frame != scan.header.frame_id or self.last_time_source != source
        if self.last_stamp is not None and not changed:
            dt = (stamp - self.last_stamp) / 1e9
            if dt == 0:
                return
        if changed or dt < 0 or dt > self.settings['reset_gap']:
            self.tracker.reset()
            dt = self.settings['default_dt']
        self.last_stamp = stamp
        self.last_frame = scan.header.frame_id
        self.last_time_source = source
        tracks = self.tracker.update(legs, dt)
        current_ids = {track.id for track in tracks}
        for track_id in self.visible_ids - current_ids:
            marker = Marker()
            marker.header = scan.header
            marker.ns = 'people'
            marker.id = track_id
            marker.action = Marker.DELETE
            self.publisher.publish(marker)
        for track in tracks:
            marker = Marker()
            marker.header = scan.header
            marker.ns = 'people'
            marker.id = track.id
            marker.type = Marker.CYLINDER if track.pair_hits >= 5 else Marker.CUBE
            marker.action = Marker.ADD
            x, y = track.position
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.25 if track.pair_hits >= 5 else 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.2
            marker.scale.z = 0.5 if track.pair_hits >= 5 else 0.2
            r, g, b = colorsys.hsv_to_rgb((track.id * 0.61803398875) % 1.0, 0.8, 1.0)
            marker.color.r, marker.color.g, marker.color.b = float(r), float(g), float(b)
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=self.settings['marker_lifetime']).to_msg()
            self.publisher.publish(marker)
        self.visible_ids = current_ids


def main(args=None):
    # Handle Ctrl+C in Python so the ROS context stays alive until cleanup.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = LegTrackerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch and the process group may both deliver SIGINT. Do not let a
        # second signal interrupt file flush/close or DDS entity destruction.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
