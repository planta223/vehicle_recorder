"""Buffered observation journal. Source messages and receipt times stay separate."""
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid

from rosidl_runtime_py.convert import message_to_ordereddict


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def flatten(value, prefix=''):
    row = {}
    for key, item in value.items():
        name = prefix + key
        if isinstance(item, dict):
            row.update(flatten(item, name + '.'))
        elif isinstance(item, (tuple, list)):
            row[name] = json.dumps(item, separators=(',', ':'), allow_nan=False)
        else:
            row[name] = item
    return row


class RunRecorder:
    def __init__(self, root, metadata, capacity=4096, snapshot_files=()):
        if capacity <= 0:
            raise ValueError('record_queue_size must be positive')
        self.directory = Path(root) / (datetime.now().strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True)
        self.metadata = dict(metadata, created_at=datetime.now().astimezone().isoformat(),
                             nonfinite_policy='JSON null; source validity is preserved',
                             clock='source_stamp_ns and received_ros_ns: ROS time; received_monotonic_ns: local monotonic',
                             complete=False, review_status='pending', record_name=None,
                             snapshots=[], snapshot_errors=[])
        for index, filename in enumerate(snapshot_files):
            try:
                source = Path(filename)
                dest = self.directory / 'config' / (str(index) + '-' + source.name)
                dest.parent.mkdir(exist_ok=True)
                shutil.copy2(source, dest)
                self.metadata['snapshots'].append(dict(source=str(source), file=str(dest.relative_to(self.directory)),
                    sha256=hashlib.sha256(dest.read_bytes()).hexdigest()))
            except OSError as error:
                self.metadata['snapshot_errors'].append(dict(source=filename, error=str(error)))
        self.pending = queue.Queue(maxsize=capacity)
        self.stopping = threading.Event()
        self.written = self.dropped = 0
        self.errors = []
        self.files = {}
        self.metadata_path = self.directory / 'metadata.json'
        self.metadata_path.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n')
        self.thread = threading.Thread(target=self._write, name='monitor-recorder', daemon=True)
        self.thread.start()

    def offer(self, item):
        if self.stopping.is_set():
            self.dropped += 1
            return False
        try:
            self.pending.put_nowait(item)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def message(self, topic, type_name, msg, received_ros_ns):
        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        source = (stamp.sec * 10**9 + stamp.nanosec if stamp is not None and
                  stamp.sec >= 0 and 0 <= stamp.nanosec < 10**9 else None)
        return self.offer(dict(kind='message', topic=topic, type=type_name, source_stamp_ns=source,
                               received_ros_ns=received_ros_ns, received_monotonic_ns=time.monotonic_ns(),
                               message=msg))

    def _write(self):
        streams, writers = {}, {}
        try:
            with (self.directory / 'messages.jsonl').open('w') as events:
                last_flush = time.monotonic()
                while not self.stopping.is_set() or not self.pending.empty():
                    try:
                        item = self.pending.get(timeout=0.2)
                    except queue.Empty:
                        item = None
                    if item is not None:
                        try:
                            item = dict(item)
                            if item['kind'] == 'message':
                                item['data'] = clean(message_to_ordereddict(item.pop('message')))
                                identity = (item['topic'], item['type'])
                                row = {key: item[key] for key in ('source_stamp_ns', 'received_ros_ns', 'received_monotonic_ns')}
                                row.update(flatten(item['data']))
                                if identity not in writers:
                                    suffix = hashlib.sha256('|'.join(identity).encode()).hexdigest()[:8]
                                    filename = 'topic-' + suffix + '.csv'
                                    stream = (self.directory / filename).open('w', newline='')
                                    streams[identity] = stream
                                    writers[identity] = csv.DictWriter(stream, fieldnames=list(row))
                                    writers[identity].writeheader()
                                    self.files[filename] = dict(topic=identity[0], type=identity[1])
                                writers[identity].writerow(row)
                            else:
                                item = clean(item)
                            events.write(json.dumps(item, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
                            self.written += 1
                        except Exception as error:
                            self.errors.append(str(error))
                        finally:
                            self.pending.task_done()
                    if time.monotonic() - last_flush >= 1.0:
                        events.flush()
                        for stream in streams.values():
                            stream.flush()
                        last_flush = time.monotonic()
        except Exception as error:
            self.errors.append(str(error))
            self.stopping.set()
        finally:
            for stream in streams.values():
                stream.close()
            self.metadata.update(written=self.written, dropped=self.dropped, errors=self.errors,
                                 pending=self.pending.qsize(), files=self.files,
                                 complete=not self.errors and not self.dropped and self.pending.empty())
            try:
                self.metadata_path.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n')
            except OSError as error:
                self.errors.append(str(error))

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=10)
        return not self.thread.is_alive() and not self.errors and not self.dropped

    @staticmethod
    def validate_name(name):
        if (not name or name in ('.', '..') or any(c in '/\\' or ord(c) < 32 or ord(c) == 127 for c in name)
                or len(name.encode('utf-8')) > 120):
            raise ValueError('빈 이름·경로 구분자·제어문자는 사용할 수 없습니다. 120바이트 이내로 입력해주세요.')

    def review(self, normal, name=None):
        if self.thread.is_alive():
            raise RuntimeError('기록 저장이 끝나기 전에는 검토할 수 없습니다.')
        if normal:
            self.validate_name(name)
        self.metadata.update(review_status='accepted' if normal else 'rejected',
                             record_name=name if normal else None,
                             reviewed_at=datetime.now().astimezone().isoformat())
        # The operator's assessment is separate from complete/dropped/errors.
        temporary = self.metadata_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n')
        temporary.replace(self.metadata_path)
        if normal:
            target = self.directory.with_name(self.directory.name + '-' + name)
            if target.exists():
                raise FileExistsError(str(target))
            self.directory.rename(target)
            self.directory = target
            self.metadata_path = target / 'metadata.json'
