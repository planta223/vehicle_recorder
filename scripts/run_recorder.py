"""Low-rate session metadata/UI; ROS payloads are handled only by the C++ backend."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import uuid

from ament_index_python.packages import get_package_share_directory


class RunRecorder:
    def __init__(self, root, metadata, snapshot_files=()):
        self.directory = Path(root) / (datetime.now().strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True)
        self.metadata_path = self.directory / 'metadata.json'
        self.metadata = dict(metadata, created_at=datetime.now().astimezone().isoformat(),
            storage='rosbag2/sqlite3', serialization='cdr (unmodified serialized ROS messages)',
            clock='bag timestamp and receipts.received_ros_ns: ROS time; receipts.received_monotonic_ns: CLOCK_MONOTONIC',
            nonfinite_policy='preserve original CDR bits, including NaN/Inf',
            complete=False, review_status='pending', record_name=None,
            acquisition_lossless_verified=False,
            completeness_scope='complete checks local storage and reported DDS errors; it does not prove upstream delivery',
            snapshots=[], snapshot_errors=[], interfaces=[], interface_errors=[])
        self.closed = False
        self.interface_packages = set()
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
        self.save()

    def save(self):
        temporary = self.metadata_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n')
        temporary.replace(self.metadata_path)

    def preserve_interfaces(self, type_names):
        """Archive installed definitions and dependencies once, outside the acquisition process."""
        pending = [name.split('/')[0] for name in type_names]
        while pending:
            package = pending.pop()
            if package in self.interface_packages:
                continue
            self.interface_packages.add(package)
            try:
                share = Path(get_package_share_directory(package))
                xml = share / 'package.xml'
                # A dependency may be a plain library with no message definitions.
                definitions = [d for d in ('msg', 'srv', 'action') if (share / d).is_dir()]
                if not definitions:
                    if package in [t.split('/')[0] for t in type_names]:
                        raise FileNotFoundError('No installed interface definitions: ' + package)
                    continue
                files = [xml] + [p for d in definitions for p in (share / d).rglob('*')
                                 if p.is_file() and p.suffix in ('.msg', '.idl', '.srv', '.action')]
                for source in files:
                    dest = self.directory / 'interfaces' / package / source.relative_to(share)
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source, dest)
                    self.metadata['interfaces'].append(dict(file=str(dest.relative_to(self.directory)),
                        sha256=hashlib.sha256(dest.read_bytes()).hexdigest()))
                    if source.suffix in ('.msg', '.srv', '.action'):
                        for line in source.read_text().splitlines():
                            match = re.match(r'\s*([A-Za-z]\w*)/', line)
                            if match:
                                pending.append(match[1])
                    elif source.suffix == '.idl':
                        pending.extend(re.findall(r'#include\s+"([A-Za-z]\w*)/', source.read_text()))
            except Exception as error:
                self.metadata['interface_errors'].append(dict(package=package, error=str(error)))

    def close(self, report, returncode):
        self.closed = True
        self.metadata['backend'] = report
        self.metadata['backend_returncode'] = returncode
        self.metadata['undiscovered_topics'] = sorted(set(report.get('requested_topics', [])) -
                                                     set(report.get('topics', {})))
        self.metadata['written'] = report.get('written', 0)
        self.metadata['dropped'] = report.get('queue_overflow', 0)
        self.metadata['errors'] = report.get('errors', [])
        self.metadata['complete'] = bool(returncode == 0 and report.get('complete') and
            not self.metadata['snapshot_errors'] and not self.metadata['interface_errors'])
        self.save()
        return self.metadata['complete']

    @staticmethod
    def validate_name(name):
        if (not name or name in ('.', '..') or any(c in '/\\' or ord(c) < 32 or ord(c) == 127 for c in name)
                or len(name.encode('utf-8')) > 120):
            raise ValueError('빈 이름·경로 구분자·제어문자는 사용할 수 없습니다. 120바이트 이내로 입력해주세요.')

    def review(self, normal, name=None):
        if not self.closed:
            raise RuntimeError('기록 저장이 끝나기 전에는 검토할 수 없습니다.')
        if normal:
            self.validate_name(name)
        target = self.directory.with_name(self.directory.name + '-' + name) if normal else self.directory
        if normal and target.exists():
            raise FileExistsError(str(target))
        self.metadata.update(review_status='accepted' if normal else 'rejected',
                             record_name=name if normal else None,
                             reviewed_at=datetime.now().astimezone().isoformat())
        self.save()
        if normal:
            self.directory.rename(target)
            self.directory = target
            self.metadata_path = target / 'metadata.json'
