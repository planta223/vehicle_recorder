"""Preserve actual controller XY reference geometry once per content change."""
import hashlib
import json
import time

from nav_msgs.msg import Path
from rclpy.qos import qos_profile_sensor_data
from run_recorder import clean


class PathMonitor:
    def __init__(self, node, topic, recorder):
        self.node, self.topic, self.recorder = node, topic, recorder
        self.previous = None
        self.subscription = node.create_subscription(Path, topic, self.receive, qos_profile_sensor_data)

    def receive(self, message):
        geometry = clean(dict(frame_id=message.header.frame_id,
                        points=[[p.pose.position.x, p.pose.position.y] for p in message.poses]))
        finite = all(v is not None for p in geometry['points'] for v in p)
        key = (geometry['frame_id'], geometry['points'])
        if key == self.previous:
            return
        identity = hashlib.sha256(json.dumps(geometry, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        stamp = message.header.stamp
        item = dict(kind='reference_path', topic=self.topic, sha256=identity,
                    finite=finite, source_stamp_ns=stamp.sec*10**9+stamp.nanosec if stamp.sec >= 0 and 0 <= stamp.nanosec < 10**9 else None,
                    received_ros_ns=self.node.get_clock().now().nanoseconds,
                    received_monotonic_ns=time.monotonic_ns(), **geometry)
        if self.recorder.offer(item):
            self.previous = key
