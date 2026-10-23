#!/usr/bin/env python3
"""Run only in an isolated ROS domain/container; no hardware, live=false launch."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import termios
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import Float64


assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '181'
OUT = Path(os.environ.get('RECORD_TERMINAL_TEST_OUT', '/evidence/terminal'))
OUT.mkdir(parents=True, exist_ok=True)


class TerminalProcess:
    def __init__(self, command, label):
        self.label = label
        self.output = b''
        self.pid, self.fd = pty.fork()
        if self.pid == 0:
            os.execvpe(command[0], command, dict(os.environ, ROS_LOG_DIR=str(OUT / label / 'ros')))
        self.original = termios.tcgetattr(self.fd)
        self.status = None

    def send(self, value):
        os.write(self.fd, value.encode('utf-8'))

    def read(self):
        if select.select([self.fd], [], [], .03)[0]:
            try:
                self.output += os.read(self.fd, 65536)
            except OSError:
                pass

    def wait_for(self, text, spin, timeout=15):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            spin()
            self.read()
            if text.encode('utf-8') in self.output:
                return
        raise AssertionError(f'{self.label}: missing {text!r}: {self.output.decode(errors="replace")}')

    def finish(self, spin):
        start = time.monotonic()
        while time.monotonic() - start < 15:
            spin()
            self.read()
            pid, status = os.waitpid(self.pid, os.WNOHANG)
            if pid:
                self.status = status
                assert os.waitstatus_to_exitcode(status) == 0, self.output.decode(errors='replace')
                assert termios.tcgetattr(self.fd)[3] == self.original[3], 'terminal flags not restored'
                return
        raise AssertionError('process failed to exit: ' + self.output.decode(errors='replace'))

    def close(self):
        if self.status is None:
            os.killpg(self.pid, signal.SIGKILL)
            os.waitpid(self.pid, 0)
        (OUT / (self.label + '.log')).write_bytes(self.output)
        os.close(self.fd)


def metadata(label):
    paths = list((OUT / label / 'runs').glob('*/metadata.json'))
    assert len(paths) == 1, paths
    return paths[0], json.loads(paths[0].read_text())


def run():
    rclpy.init()
    fixture = rclpy.create_node('record_terminal_fixture')
    pub = fixture.create_publisher(Float64, '/recorder_terminal_fixture', 10)
    fixture.create_timer(.02, lambda: pub.publish(Float64(data=42.0)))
    spin = lambda: rclpy.spin_once(fixture, timeout_sec=.01)
    executable = get_package_prefix('vehicle_recorder') + '/lib/vehicle_recorder/record'

    def command(label):
        return ['ros2', 'run', 'vehicle_recorder', 'record', '--ros-args',
                '-p', 'log_root:=' + str(OUT / label / 'runs'),
                '-p', "topics:=['/recorder_terminal_fixture']"]

    try:
        for label, answer in [('accepted', 'y'), ('rejected', 'n')]:
            process = TerminalProcess(command(label), label)
            try:
                process.wait_for('q: 기록 종료', spin)
                end = time.monotonic() + 1.5
                while time.monotonic() < end:
                    spin()
                process.send('Q' if answer == 'n' else 'q')
                process.wait_for('정상적인 기록입니까? y/n:', spin)
                path, meta = metadata(label)
                journal_at_prompt = (path.parent / 'messages.jsonl').read_bytes()
                rows = [json.loads(line) for line in journal_at_prompt.splitlines()]
                assert any(row.get('data', {}).get('data') == 42.0 for row in rows), rows
                assert meta['complete'] and meta['review_status'] == 'pending'
                if answer == 'y':
                    process.send('invalid\n')
                    process.wait_for('y 또는 n을 입력해주세요.', spin)
                process.send(answer.upper() + '\n')
                if answer == 'y':
                    process.wait_for('기록 이름을 입력해주세요.', spin)
                    process.send('\n../escape\n')
                    process.wait_for('120바이트 이내로 입력해주세요.', spin)
                    process.send('모터 RPM 1차\n')
                process.finish(spin)
                path, meta = metadata(label)
                assert meta['review_status'] == ('accepted' if answer == 'y' else 'rejected')
                assert meta['record_name'] == ('모터 RPM 1차' if answer == 'y' else None)
                assert (path.parent / 'messages.jsonl').read_bytes() == journal_at_prompt
                if answer == 'n':
                    assert '기록 이름을 입력해주세요.' not in process.output.decode()
                print('PASS', label, 'data flushed before prompt and preserved', flush=True)
            finally:
                process.close()

        label = 'launch'
        process = TerminalProcess(['ros2', 'launch', 'manual_control', 'brake_test_support.launch.py',
            'live:=false', 'log_root:=' + str(OUT / label / 'runs')], label)
        try:
            process.wait_for('q: 기록 종료', spin)
            process.send('q')
            process.wait_for('정상적인 기록입니까? y/n:', spin)
            process.send('y\n')
            process.wait_for('기록 이름을 입력해주세요.', spin)
            process.send('제동 시험\n')
            process.finish(spin)
            path, meta = metadata(label)
            assert meta['record_name'] == '제동 시험' and path.parent.name.endswith('-제동 시험')
            assert all((path.parent / s['file']).exists() for s in meta['snapshots'])
            assert 'trans-3]: process has finished cleanly' in process.output.decode(), process.output.decode()
            print('PASS launch /dev/tty review, snapshots and support shutdown', flush=True)
        finally:
            process.close()

        for label, during_review in [('sigint', False), ('review-signal', True)]:
            # Direct executable gives the signal check a single well-defined PID.
            process = TerminalProcess([executable] + command(label)[4:], label)
            try:
                process.wait_for('q: 기록 종료', spin)
                if during_review:
                    process.send('q')
                    process.wait_for('정상적인 기록입니까? y/n:', spin)
                    os.kill(process.pid, signal.SIGTERM)
                else:
                    process.send('\x03')
                process.finish(spin)
                _, meta = metadata(label)
                assert meta['complete'] and meta['review_status'] == 'pending'
                print('PASS', label, 'pending review, clean shutdown, terminal restored', flush=True)
            finally:
                process.close()

        label = 'no-tty'
        with (OUT / (label + '.log')).open('w') as log:
            process = subprocess.Popen([executable] + command(label)[4:], stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                end = time.monotonic() + 10
                while not list((OUT / label / 'runs').glob('*/metadata.json')):
                    assert process.poll() is None
                    assert time.monotonic() < end
                    spin()
                process.send_signal(signal.SIGTERM)
                assert process.wait(timeout=12) == 0
                _, meta = metadata(label)
                assert meta['complete'] and meta['review_status'] == 'pending'
                print('PASS no controlling terminal: unattended signal shutdown', flush=True)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
    finally:
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
