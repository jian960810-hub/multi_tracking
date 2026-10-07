#!/usr/bin/env python3
"""Record all rays as angle/range text with scan boundaries and timestamps."""
from datetime import datetime
import math
import signal
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def write_scan(stream, scan):
    """Comment lines preserve boundaries; np.loadtxt still reads the 2 columns."""
    frame = scan.header.frame_id.replace('\n', ' ').replace('\r', ' ')
    stream.write(f'# scan {scan.header.stamp.sec}.{scan.header.stamp.nanosec:09d}'
                 f' frame={frame} count={len(scan.ranges)}'
                 f' range_min={scan.range_min} range_max={scan.range_max}\n')
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + scan.angle_increment * i
        stream.write(f'{angle:.17g}    {float(distance):.17g}\n')
    stream.flush()


class ScanRecorder(Node):
    def __init__(self, **kwargs):
        super().__init__('scan_recorder', **kwargs)
        for name, value in {'scan_topic': '/scan', 'output_file': '', 'duration_seconds': 360.0}.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.duration = self.get_parameter('duration_seconds').value
        if not math.isfinite(self.duration) or self.duration < 0:
            raise ValueError('duration_seconds must be finite and >= 0 (0 = until Ctrl+C)')
        output = self.get_parameter('output_file').value
        if not output:
            output = str(Path.home() / 'multi_tracking_data' /
                         f"laser_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.txt")
        self.path = Path(output).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', encoding='utf-8')
        self.started_at = None
        self.finished = False
        self.scan_count = 0
        self.subscription = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.check_deadline,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Recording to {self.path}; duration starts with first scan')

    def on_scan(self, scan):
        if self.finished:
            return
        if self.started_at is None:
            self.started_at = time.monotonic()
        if self.duration and time.monotonic() - self.started_at >= self.duration:
            self.finish()
            return
        write_scan(self.stream, scan)
        self.scan_count += 1

    def check_deadline(self):
        if (not self.finished and self.started_at is not None and self.duration
                and time.monotonic() - self.started_at >= self.duration):
            self.finish()

    def finish(self):
        if not self.finished:
            self.stream.close()
            self.finished = True
            self.timer.cancel()
            self.get_logger().info(f'Saved {self.scan_count} scans to {self.path}')

    def destroy_node(self):
        self.finish()
        return super().destroy_node()


def main(args=None):
    # Handle Ctrl+C in Python so the ROS context stays alive until cleanup.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ScanRecorder()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
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
