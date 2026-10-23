"""Subscriber-only data recording; never waits in a control callback for disk or web I/O."""
import hashlib
import os
import signal
import threading
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from run_recorder import RunRecorder
from path_monitor import PathMonitor


TOPICS = ['/gnss/odom', '/control/global_pose', '/control/motion_state',
          '/control/pp_diagnostic', '/control/auto_cmd', '/control/selected_cmd',
          '/control/actuator_raw', '/control/status', '/control/stop_status',
          '/vehicle/stop_status', '/vehicle/command_events', '/motor_feedback', '/steering_feedback']


class RecorderNode(Node):
    def __init__(self):
        super().__init__('vehicle_recorder', enable_rosout=False)
        self.topics = self.declare_parameter('topics', TOPICS).value
        root = self.declare_parameter('log_root', '/workspace/log/vehicle_recorder').value
        capacity = self.declare_parameter('record_queue_size', 4096).value
        path = self.declare_parameter('reference_path_topic', '/control/reference_path').value
        snapshots = self.declare_parameter('snapshot_files', ['']).value
        source = Path(__file__).resolve().parent
        self.recorder = RunRecorder(root, dict(
            node=self.get_name(), topics=list(self.topics), reference_path_topic=path,
            record_queue_size=capacity, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '0'),
            docker_image_id=os.environ.get('CONTROL_IMAGE_ID'),
            container_id=os.environ.get('CONTROL_CONTAINER_ID', os.environ.get('HOSTNAME')),
            source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.glob('*.py')}),
            capacity=capacity, snapshot_files=[p for p in snapshots if p])
        self.path_monitor = PathMonitor(self, path, self.recorder)
        self.subscriptions_by_topic = {}
        self.last_health = (0, 0, True)
        self.create_timer(.5, self.discover)
        self.create_timer(1., self.health)
        self.discover()
        self.get_logger().info('기록 폴더: ' + str(self.recorder.directory))

    def discover(self):
        graph = dict(self.get_topic_names_and_types())
        for topic in self.topics:
            if topic in self.subscriptions_by_topic:
                continue
            types = graph.get(topic, [])
            if len(types) != 1:
                continue
            try:
                cls = get_message(types[0])
            except (ImportError, AttributeError, ValueError):
                continue
            type_name = types[0]
            self.subscriptions_by_topic[topic] = self.create_subscription(
                cls, topic, lambda msg, t=topic, typ=type_name:
                    self.recorder.message(t, typ, msg, self.get_clock().now().nanoseconds),
                qos_profile_sensor_data)

    def health(self):
        state = (self.recorder.dropped, len(self.recorder.errors), self.recorder.thread.is_alive())
        if state != self.last_health and (state[0] or state[1] or not state[2]):
            self.get_logger().error(f'Recording incomplete: dropped={state[0]} errors={state[1]} writer_alive={state[2]}')
        self.last_health = state


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())
    node = None
    complete = True
    try:
        node = RecorderNode()
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        if node is not None:
            node.recorder.errors.append('Recorder process failed: ' + str(error))
        raise
    finally:
        if node is not None:
            complete = node.recorder.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not complete:
        raise SystemExit(1)
