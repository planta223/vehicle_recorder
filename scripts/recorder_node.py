"""Session launcher. No high-rate ROS subscriptions or Python message conversion."""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import yaml

from run_recorder import RunRecorder
from record_terminal import RecordTerminal

TOPICS = ['/vehicle/brake_test', '/gnss/odom', '/control/global_pose', '/control/motion_state',
          '/control/pp_diagnostic', '/control/auto_cmd', '/control/selected_cmd',
          '/control/actuator_raw', '/control/status', '/control/stop_status',
          '/vehicle/stop_status', '/vehicle/command_events', '/motor_feedback', '/steering_feedback',
          '/sensors/detectTraffic', '/control/sensors/traffic']


def read_report(directory):
    path = directory / 'backend.yaml'
    return yaml.safe_load(path.read_text()) if path.exists() else {}


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())
    session = child = terminal = node = None
    quit_requested = False
    complete = False
    try:
        node = Node('vehicle_recorder', enable_rosout=False)
        values = {name: node.declare_parameter(name, default).value for name, default in dict(
            topics=TOPICS, log_root='/workspace/log/vehicle_recorder', record_queue_size=4096,
            record_queue_bytes=268435456, subscription_depth=4096, interactive_review=True,
            reference_path_topic='/control/reference_path', snapshot_files=['']).items()}
        backend = Path(get_package_prefix('vehicle_recorder')) / 'lib/vehicle_recorder/record_backend'
        source = Path(__file__).resolve().parent
        session = RunRecorder(values['log_root'], dict(
            node=node.get_name(), effective_parameters=values, topics=list(values['topics']),
            reference_path_topic=values['reference_path_topic'],
            ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '0'),
            docker_image_id=os.environ.get('CONTROL_IMAGE_ID'),
            container_id=os.environ.get('CONTROL_CONTAINER_ID', os.environ.get('HOSTNAME')),
            source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in [*source.glob('*.py'), backend] if p.is_file()}),
            snapshot_files=[p for p in values['snapshot_files'] if p])
        # Root/default config and CLI overrides have already been resolved by rclpy.
        parameters = {name: values[name] for name in ('topics', 'reference_path_topic',
                      'record_queue_size', 'record_queue_bytes', 'subscription_depth')}
        parameters['session_directory'] = str(session.directory.resolve())
        config = session.directory / 'backend_parameters.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
        node.destroy_node()
        node = None
        rclpy.shutdown()
        # Retain caller ROS remappings; explicit effective values are applied last.
        child = subprocess.Popen([str(backend), *sys.argv[1:], '--ros-args', '--params-file', str(config)],
                                 start_new_session=True)
        # Announce the UI only after the backend installed its signal handlers and opened storage.
        # An immediate q/Ctrl+C must not kill an uninitialized child before it can close the bag.
        while child.poll() is None and not (session.directory / 'backend.yaml').exists():
            time.sleep(.05)
        terminal = RecordTerminal(values['interactive_review'])
        text = '기록 폴더: ' + str(session.directory) + '\nq: 기록 종료 및 정상 여부 확인\n'
        if terminal.fd is not None:
            terminal.write(text)
        else:
            print(text, flush=True)
        last_report_mtime = None
        while child.poll() is None and not stopping.is_set():
            if terminal.quit_requested():
                quit_requested = True
                break
            status = session.directory / 'backend.yaml'
            modified = status.stat().st_mtime_ns if status.exists() else None
            if modified != last_report_mtime:
                report = read_report(session.directory)
                session.preserve_interfaces([t['type'] for t in report.get('topics', {}).values()])
                last_report_mtime = modified
            stopping.wait(.1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if terminal is not None:
            terminal.restore()
        if child is not None:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            # Drain every queued buffer; never kill the writer after an arbitrary timeout.
            child.wait()
        try:
            if session is not None:
                report = read_report(session.directory)
                session.preserve_interfaces([t['type'] for t in report.get('topics', {}).values()])
                complete = session.close(report, child.returncode if child is not None else -1)
                if quit_requested and not stopping.is_set():
                    if not complete:
                        terminal.write('\n주의: 기록 누락 또는 저장 오류가 있습니다. metadata.json을 확인해주세요.\n')
                    terminal.review(session, stopping)
        finally:
            if terminal is not None:
                terminal.close()
    if not complete:
        raise SystemExit(1)
