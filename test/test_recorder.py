#!/usr/bin/env python3
"""Session review never changes the raw bag, and failures cannot be accepted as complete."""
import json
import sys
from pathlib import Path
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from run_recorder import RunRecorder


class RecorderTest(unittest.TestCase):
    def test_review_preserves_bag_and_snapshots(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'settings.yaml'
            config.write_text('fixture: true\n')
            session = RunRecorder(directory, {}, snapshot_files=[str(config)])
            bag = session.directory / 'bag'
            bag.mkdir()
            (bag / 'fixture.db3').write_bytes(b'\x00\xffraw CDR')
            self.assertTrue(session.close({'complete': True, 'written': 1}, 0))
            original = session.directory
            session.review(True, '모터 RPM 1차')
            self.assertFalse(original.exists())
            self.assertEqual((session.directory / 'bag/fixture.db3').read_bytes(), b'\x00\xffraw CDR')
            meta = json.loads(session.metadata_path.read_text())
            self.assertEqual(meta['review_status'], 'accepted')
            self.assertTrue((session.directory / meta['snapshots'][0]['file']).exists())
            self.assertFalse(meta['acquisition_lossless_verified'])

    def test_failures_and_review_are_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            session = RunRecorder(directory, {})
            with self.assertRaises(RuntimeError):
                session.review(False)
            self.assertFalse(session.close({'complete': False, 'queue_overflow': 1}, 1))
            original = session.directory
            session.review(False)
            self.assertEqual(original, session.directory)
            self.assertEqual(session.metadata['review_status'], 'rejected')
            self.assertFalse(session.metadata['complete'])

    def test_invalid_names_leave_run_unreviewed(self):
        with tempfile.TemporaryDirectory() as directory:
            session = RunRecorder(directory, {})
            session.close({'complete': True}, 0)
            for name in ('', '.', '..', '../escape', 'a/b', 'a\\b', 'a\x00b', 'a\nb', '가' * 41):
                with self.subTest(name=name), self.assertRaises(ValueError):
                    session.review(True, name)
            self.assertEqual(json.loads(session.metadata_path.read_text())['review_status'], 'pending')

    def test_snapshot_failure_marks_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            session = RunRecorder(directory, {}, snapshot_files=['/no-such-fixture/settings.yaml'])
            self.assertFalse(session.close({'complete': True}, 0))
            self.assertTrue(session.metadata['snapshot_errors'])

    def test_interface_dependencies_are_archived(self):
        with tempfile.TemporaryDirectory() as directory:
            session = RunRecorder(directory, {})
            session.preserve_interfaces(['nav_msgs/msg/Odometry'])
            self.assertFalse(session.metadata['interface_errors'])
            self.assertTrue((session.directory / 'interfaces/nav_msgs/msg/Odometry.msg').exists())
            self.assertTrue((session.directory / 'interfaces/geometry_msgs/msg/Point.msg').exists())
            self.assertTrue((session.directory / 'interfaces/builtin_interfaces/msg/Time.msg').exists())


if __name__ == '__main__':
    unittest.main()
