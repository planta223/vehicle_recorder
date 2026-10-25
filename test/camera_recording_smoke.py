#!/usr/bin/env python3
"""Run only in a disposable, network-isolated container; never starts hardware."""
from array import array
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
import yaml

assert os.environ.get('VEHICLE_RECORDER_ISOLATED_TEST') == '1'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
OUT = Path('/evidence')
EXE = Path(get_package_prefix('vehicle_recorder')) / 'lib/vehicle_recorder/record'
CAMERA = '/camera/image_raw/compressed'
children = []
logs = []


def start(args, name, env=None):
    log = (OUT / (name + '.log')).open('w')
    logs.append(log)
    proc = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    children.append(proc)
    return proc


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
    proc.wait(timeout=30)


def wait(node, predicate, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.005)
        if predicate():
            return
    raise AssertionError('Timed out')


def session(root):
    files = list(root.glob('*/metadata.json'))
    assert len(files) == 1, files
    return files[0].parent, json.loads(files[0].read_text())


def messages(run):
    with sqlite3.connect((run/'bag/bag_0.db3').resolve().as_uri()+'?mode=ro', uri=True) as db:
        return db.execute('select t.name,m.timestamp,m.data from messages m '
                          'join topics t on t.id=m.topic_id order by m.id').fetchall()


def main():
    # Missing publisher must fail before creating any camera bag.
    env = dict(os.environ, TEST_ROOT=str(OUT/'missing'))
    proc = start(['bash', '/workspace/tools/record_camera.sh', '-p',
                  'interactive_review:=false'], 'missing-publisher', env)
    assert proc.wait(timeout=15) != 0
    assert not list((OUT/'missing').glob('camera_records/*/bag'))

    rclpy.init()
    node = rclpy.create_node('camera_recording_fixture')
    cam = node.create_publisher(CompressedImage, CAMERA, qos_profile_sensor_data)
    control = node.create_publisher(Float64, '/camera_test/control', 10)
    control_proc = start([str(EXE), '--ros-args', '-p', 'log_root:='+str(OUT/'control'),
        '-p', 'interactive_review:=false', '-p', "reference_path_topic:=''",
        '-p', "topics:=['/camera_test/control']", '-r', '__node:=control_test_recorder'], 'control')
    env = dict(os.environ, TEST_ROOT=str(OUT/'normal'))
    # The helper must supply the normal main-container profile if absent.
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    camera_proc = start(['bash', '/workspace/tools/record_camera.sh', '-p',
                         'interactive_review:=false'], 'camera', env)
    wait(node, lambda: cam.get_subscription_count() == 1 and control.get_subscription_count() == 1)
    # Camera helper and its descendants inherit low CPU priority.
    assert os.getpriority(os.PRIO_PROCESS, camera_proc.pid) >= 10
    config = yaml.safe_load(Path('/workspace/src/vehicle_recorder/config/record_camera.yaml').read_text())['/**']['ros__parameters']
    assert config['topics'] == [CAMERA] and config['reference_path_topic'] == ''
    assert config['record_queue_size'] == 64 and config['record_queue_bytes'] == 33554432
    assert config['subscription_depth'] == 8
    duplicate = start(['bash', '/workspace/tools/record_camera.sh', '-p',
                       'interactive_review:=false'], 'duplicate', env)
    assert duplicate.wait(timeout=5) != 0
    assert camera_proc.poll() is None
    rng = np.random.default_rng(25)
    image = rng.integers(0,256,(720,1280,3),dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY,80])
    assert ok
    jpeg = encoded.tobytes()
    camera_expected = []
    control_expected = []

    def drive(seconds, record_camera):
        origin = time.monotonic()
        ci = vi = 0
        while time.monotonic()-origin < seconds:
            now = time.monotonic()-origin
            if ci/50 <= now:
                msg = Float64(data=float(len(control_expected)))
                payload = serialize_message(msg)
                control.publish(payload)
                control_expected.append(payload)
                ci += 1
            if vi/30 <= now:
                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_optical_frame'
                msg.format = 'jpeg'
                msg.data = array('B', jpeg)
                payload = serialize_message(msg)
                cam.publish(payload)
                if record_camera:
                    camera_expected.append(payload)
                vi += 1
            rclpy.spin_once(node, timeout_sec=.001)

    drive(4, True)
    time.sleep(.5)
    stop(camera_proc)
    assert control_proc.poll() is None
    wait(node, lambda: cam.get_subscription_count() == 0)
    normal_run, meta = session(OUT/'normal/camera_records')
    assert meta['complete'] and not meta['undiscovered_topics'], meta
    assert meta['effective_parameters']['reference_path_topic'] == ''
    assert meta['effective_parameters']['record_queue_size'] == 64
    assert not meta['snapshot_errors'] and not meta['interface_errors']
    rows = messages(normal_run)
    assert len(rows) == len(camera_expected)
    for (topic, stamp, payload), expected in zip(rows, camera_expected):
        assert topic == CAMERA and payload == expected
        decoded = deserialize_message(payload, CompressedImage)
        assert bytes(decoded.data) == jpeg
        assert stamp >= decoded.header.stamp.sec*10**9+decoded.header.stamp.nanosec
    assert meta['backend']['received'] == meta['backend']['written'] == len(rows)
    assert (normal_run/'interfaces/sensor_msgs/msg/CompressedImage.msg').exists()

    # Deliberate camera queue failure must leave the control recorder running.
    env = dict(os.environ, TEST_ROOT=str(OUT/'overflow'))
    failed = start(['bash','/workspace/tools/record_camera.sh','-p','interactive_review:=false',
                    '-p','record_queue_bytes:=1'], 'camera-overflow', env)
    wait(node, lambda: cam.get_subscription_count() == 1)
    drive(1, False)
    assert failed.wait(timeout=20) != 0
    assert control_proc.poll() is None
    failed_run, failed_meta = session(OUT/'overflow/camera_records')
    assert not failed_meta['complete'] and failed_meta['dropped'] > 0
    assert failed_meta['errors']
    drive(1, False)
    time.sleep(.5)
    stop(control_proc)
    control_run, control_meta = session(OUT/'control')
    assert control_meta['complete'], control_meta
    assert [r[2] for r in messages(control_run)] == control_expected
    assert set(control_meta['backend']['topics']) == {'/camera_test/control'}
    result = dict(camera_frames=len(camera_expected), control_messages=len(control_expected),
        jpeg_bytes_per_frame=len(jpeg), jpeg_and_cdr_unchanged=True,
        original_header_timestamps_preserved=True, separate_topics_and_bags=True,
        low_cpu_priority=True, missing_camera_rejected=True,
        duplicate_recording_rejected=True, default_shm_profile_applied=True,
        camera_overflow_isolated=True, control_recording_complete=True,
        field_timing_verified=False)
    (OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    finally:
        for child in children:
            stop(child)
        for log in logs:
            log.close()
