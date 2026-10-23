#!/usr/bin/env python3
"""고규범 신규 검증 (2026-10-21): isolated ROS + loopback UDP, no hardware devices."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time

assert os.environ.get('ROS_DOMAIN_ID') == '179'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('CONTROL_CONSOLE_ISOLATED_TEST') == '1'

import rclpy
from bag_helpers import read_bag
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from vehicle_recorder.msg import CommandEvent, StopStatus
from manual_control import manual_control_node as manual_module
from trans.trans_node import TransNode


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class FakeInput:
    steering, speed, brake = 0.375, 0.4, 0
    device_path, help_text = 'fixture-no-device', 'isolated fixture'
    def start(self): pass
    def stop(self): pass
    def update(self): pass


def run():
    out = Path(os.environ.get('STOP_MONITOR_TEST_OUT', '/evidence/stop-monitor'))
    out.mkdir(parents=True, exist_ok=True)
    prior_journals = set((out / 'runs').glob('*/metadata.json'))
    env = dict(os.environ, ROS_LOG_DIR=str(out / 'ros'))
    log = (out / 'monitor.log').open('w')
    process = subprocess.Popen([get_package_prefix('vehicle_recorder') + '/lib/vehicle_recorder/stop_observer',
                                '--ros-args'],
                               env=env, stdout=log, stderr=subprocess.STDOUT)
    record_log = (out / 'record.log').open('w')
    recorder = subprocess.Popen([get_package_prefix('vehicle_recorder') + '/lib/vehicle_recorder/record',
        '--ros-args', '-p', 'log_root:=' + str(out / 'runs')], env=env, stdout=record_log, stderr=subprocess.STDOUT)
    fixture = rclpy.create_node('stop_monitor_fixture')
    values, events = {}, []
    fixture.create_subscription(StopStatus, '/vehicle/stop_status', lambda m: values.update(status=m), 10)
    fixture.create_subscription(Float32MultiArray, '/cmd', lambda m: values.update(raw=list(m.data)), 10)
    fixture.create_subscription(CommandEvent, '/vehicle/command_events', events.append, 100)
    publisher = fixture.create_publisher(Odometry, '/gnss/odom', 10)
    state = dict(speed=1.0, frozen=False, invalid=False)
    old_stamp = None

    def odometry():
        nonlocal old_stamp
        m = Odometry()
        m.header.frame_id, m.child_frame_id = 'map', 'base_link'
        if not state['frozen'] or old_stamp is None:
            old_stamp = fixture.get_clock().now().to_msg()
        m.header.stamp = old_stamp
        m.twist.twist.linear.x = state['speed']
        m.twist.covariance[0] = 1e12 if state['invalid'] else 0.01
        m.pose.pose.position.x, m.pose.pose.position.y = 10.0, 5.0
        m.pose.covariance[0] = m.pose.covariance[7] = 0.01
        publisher.publish(m)

    fixture.create_timer(0.05, odometry)
    device = FakeInput()
    manual_module.KeyboardInput = lambda **kwargs: device
    manual = manual_module.ManualControlNode()  # FakeInput.start never opens /dev/input.
    trans = TransNode()
    sockets = []
    for _ in range(3):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('127.0.0.1', 0))
        s.setblocking(False)
        sockets.append(s)
    # The only UDP destinations in this process are three ephemeral loopback sockets.
    trans._addrs = tuple(s.getsockname() for s in sockets)
    trans._test_mode = False
    executor = rclpy.executors.SingleThreadedExecutor()
    for node in (fixture, manual, trans): executor.add_node(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    def wait(predicate, label, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
            assert process.poll() is None, (out / 'monitor.log').read_text()
            if predicate():
                print('\nPASS:', label, flush=True)
                return
        raise AssertionError(label)

    try:
        wait(lambda: 'status' in values and values['status'].speed_valid and
             not values['status'].stopped, 'moving GNSS status is reusable without an active stop')
        device.brake = 1
        device.steering = -0.9  # Input changes during brake must not recenter/change the output.
        wait(lambda: values.get('raw') == [0.375, 0.0, 1.0] and
             values['status'].command_active and ns(values['status'].transmit_stamp) > 0,
             'manual output holds steering; real loopback send has its own timestamp')
        # Drain older driving packets, then verify every board gets held steer / zero motor / bit 7.
        for s in sockets:
            while True:
                try: s.recv(100)
                except BlockingIOError: break
        spin(0.15)
        for s in sockets:
            packet = s.recv(100)
            # Current shared wire contract: float32 degrees, float32 RPM, uint8 misc.
            degrees, rpm, misc = struct.unpack('<ffB', packet)
            assert math.isclose(degrees, -6.37875, abs_tol=1e-5)
            assert rpm == 0.0 and misc == 128
        first_command = ns(values['status'].command_stamp)
        first_transmit = ns(values['status'].transmit_stamp)
        assert first_command > 0 and first_transmit >= first_command
        state.update(speed=0.0, frozen=True)
        spin(0.7)
        assert not values['status'].stopped and not values['status'].speed_valid
        print('\nPASS: repeated stale GNSS cannot fill the half-second window', flush=True)
        state.update(frozen=False)
        wait(lambda: values['status'].stopped, 'distinct GNSS samples confirm stop')
        m = values['status']
        assert ns(m.confirmed_at) - ns(m.stopped_since) >= 500_000_000
        assert ns(m.command_stamp) == first_command and ns(m.transmit_stamp) == first_transmit
        assert m.command_position_valid and m.transmit_position_valid and m.stop_position_valid
        assert m.distance_valid and math.isclose(m.distance_m, 0.0)
        spin(0.3)
        recorder.send_signal(signal.SIGINT)
        assert recorder.wait(timeout=8) == 0
        print('PASS: recording stopped; control and observation continue', flush=True)
        state['invalid'] = True
        wait(lambda: not values['status'].speed_valid and not values['status'].stopped,
             'invalid speed covariance revokes stopped status')
        state.update(invalid=False, speed=1.0)
        device.brake, device.speed = 0, 0.2
        wait(lambda: not values['status'].command_active, 'command release ends the episode')
        trans._test_mode = True
        device.brake = 1
        wait(lambda: values['status'].command_active and values['status'].stop_id == 2,
             'next braking episode gets a new stop id')
        spin(0.2)
        assert ns(values['status'].command_stamp) > first_command
        assert ns(values['status'].transmit_stamp) == 0 and not values['status'].distance_valid
        assert any(m.stage == CommandEvent.DRY_RUN for m in events)
        print('\nPASS: TEST_MODE reports DRY_RUN and never fabricates a transmit time', flush=True)
    finally:
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=8)
        record_log.close()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=8)
        for node in (fixture, manual, trans): node.destroy_node()
        executor.shutdown()
        for s in sockets: s.close()
        log.close()
    files = [p for p in (out / 'runs').glob('*/metadata.json') if p not in prior_journals]
    assert files
    journal = read_bag(files[0].parent)
    commands = [e['data'] for e in journal if e.get('topic') == '/vehicle/command_events']
    assert any(e['stage'] == CommandEvent.COMMAND and e['brake'] for e in commands)
    assert any(e['stage'] == CommandEvent.TRANSMITTED and e['brake'] for e in commands)
    stopped = [e['data'] for e in journal if e.get('topic') == '/vehicle/stop_status' and e['data']['stopped']]
    assert stopped
    first = stopped[-1]
    assert ns(type('Stamp', (), first['stopped_since'])()) < ns(type('Stamp', (), first['confirmed_at'])())
    print('ALL STOP MONITOR SMOKE CHECKS PASSED', flush=True)


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
