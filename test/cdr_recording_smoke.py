#!/usr/bin/env python3
"""Isolated integration check: exact CDR bytes, all samples, timestamps, and explicit failures."""
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from std_msgs.msg import Float64
import yaml

assert os.environ.get('VEHICLE_RECORDER_ISOLATED_TEST') == '1'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
OUT = Path(os.environ.get('CDR_TEST_OUT', '/evidence/cdr'))
OUT.mkdir(parents=True, exist_ok=True)
EXE = Path(get_package_prefix('vehicle_recorder')) / 'lib/vehicle_recorder'


def wait(predicate, node, seconds=15):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.01)
        if predicate():
            return
    raise AssertionError('condition timed out')


def recording(node):
    topics = ['/cdr/odom', '/cdr/path', '/cdr/special']
    pubs = [node.create_publisher(cls, topic, qos_profile_sensor_data)
            for cls, topic in zip((Odometry, RosPath, Float64), topics)]
    log = (OUT / 'record.log').open('w')
    child = subprocess.Popen([str(EXE / 'record'), '--ros-args', '-p', 'log_root:=' + str(OUT / 'runs'),
        '-p', 'interactive_review:=false', '-p', 'reference_path_topic:=/cdr/path',
        '-p', 'topics:=' + json.dumps(topics)], stdout=log, stderr=subprocess.STDOUT)
    expected = collections.defaultdict(list)
    try:
        wait(lambda: all(p.get_subscription_count() for p in pubs), node)
        for seq in range(80):
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id = 'map/한글'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = seq * .123456789012345
            odom.pose.pose.position.z = -0.0
            odom.pose.covariance = [i * .0000123456789 for i in range(36)]
            odom.twist.twist.linear.x = -12.3456789012345
            odom.twist.covariance[35] = float('inf')
            path = RosPath()
            path.header = odom.header
            path.poses = [PoseStamped() for _ in range(1217)]
            for i, pose in enumerate(path.poses):
                pose.header = odom.header
                pose.pose.position.x = i + seq * .001
                pose.pose.position.y = -i * .0123456789
                pose.pose.orientation.w = 1.
            # Exact NaN payload bits, +/-Inf, -0, subnormal and high precision double.
            special = [0x7ff8000000001234, 0x7ff0000000000000, 0xfff0000000000000,
                       0x8000000000000000, 1, 0x3ff3c0ca428c59fb][seq % 6]
            payloads = [serialize_message(odom), serialize_message(path), b'\x00\x01\x00\x00' + struct.pack('<Q', special)]
            for topic, pub, payload in zip(topics, pubs, payloads):
                pub.publish(payload)
                expected[topic].append(hashlib.sha256(payload).hexdigest())
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.01)
        time.sleep(1.)
    finally:
        child.send_signal(signal.SIGINT)
        assert child.wait(timeout=60) == 0, (OUT / 'record.log').read_text()
        log.close()
        for pub in pubs:
            node.destroy_publisher(pub)
    dirs = list((OUT / 'runs').glob('*/metadata.json'))
    assert len(dirs) == 1
    run = dirs[0].parent
    meta = json.loads(dirs[0].read_text())
    assert meta['complete'], meta
    actual = collections.defaultdict(list)
    bag_rows = []
    with sqlite3.connect(str(next((run / 'bag').glob('*.db3')))) as db:
        for idx, topic, stamp, data in db.execute('SELECT m.id,t.name,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id ORDER BY m.id'):
            actual[topic].append(hashlib.sha256(data).hexdigest())
            bag_rows.append((idx, topic, stamp, len(data)))
    assert actual == expected, {t: (len(expected[t]), len(actual[t])) for t in expected}
    receipts = list(csv.DictReader((run / 'receipts.csv').open()))
    assert len(receipts) == len(bag_rows) == 240
    last_mono = 0
    for r, (idx, topic, stamp, size) in zip(receipts, bag_rows):
        assert (int(r['bag_message_index']), r['topic'], int(r['received_ros_ns']), int(r['serialized_bytes'])) == (idx, topic, stamp, size)
        assert int(r['received_monotonic_ns']) >= last_mono
        last_mono = int(r['received_monotonic_ns'])
    assert (run / 'interfaces/nav_msgs/msg/Odometry.msg').exists()
    assert (run / 'interfaces/geometry_msgs/msg/Point.msg').exists()
    return dict(messages=len(bag_rows), payload_bytes_identical=True, special_float_bits_preserved=True,
                full_path_points=1217, receipts_aligned=True, metadata_complete=meta['complete'])


def failure(node, name, disk=False):
    directory = OUT / name
    directory.mkdir()
    if disk:
        (directory / 'receipts.csv').symlink_to('/dev/full')
    pub = node.create_publisher(Odometry, '/cdr/failure', qos_profile_sensor_data)
    with (directory / 'process.log').open('w') as log:
        child = subprocess.Popen([str(EXE / 'record_backend'), '--ros-args',
            '-p', 'session_directory:=' + str(directory), '-p', "reference_path_topic:=''",
            '-p', "topics:=['/cdr/failure']", '-p', 'record_queue_bytes:=' + ('268435456' if disk else '1')],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            wait(lambda: pub.get_subscription_count() > 0, node)
            pub.publish(Odometry())
            time.sleep(.2)
            if disk:
                child.send_signal(signal.SIGINT)
            assert child.wait(timeout=15) != 0
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=15)
            node.destroy_publisher(pub)
    report = yaml.safe_load((directory / 'backend.yaml').read_text())
    assert not report['complete'] and report['errors'], report
    if not disk:
        assert report['queue_overflow'] == 1
    return dict(complete=report['complete'], errors=report['errors'])


def main():
    rclpy.init()
    node = rclpy.create_node('cdr_fixture')
    try:
        result = dict(raw=recording(node), overflow=failure(node, 'overflow'),
                      disk_failure=failure(node, 'disk-failure', disk=True))
        (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
