#!/usr/bin/env python3
"""A blocked disk writer must not block the ROS producer or hide lost samples."""
import json
import sys
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from std_msgs.msg import Float64
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from run_recorder import RunRecorder


class RecorderTest(unittest.TestCase):
    def test_named_review_preserves_messages_and_snapshot_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'settings.yaml'
            config.write_text('fixture: true\n')
            journal = RunRecorder(directory, {}, snapshot_files=[str(config)])
            journal.message('/fixture', 'std_msgs/msg/Float64', Float64(data=3.), 1)
            self.assertTrue(journal.close())
            original = journal.directory
            journal.review(True, '모터 RPM 1차')
            self.assertFalse(original.exists())
            self.assertTrue(journal.directory.name.endswith('-모터 RPM 1차'))
            meta = json.loads(journal.metadata_path.read_text())
            self.assertEqual(meta['review_status'], 'accepted')
            self.assertEqual(meta['record_name'], '모터 RPM 1차')
            self.assertTrue(meta['complete'])
            self.assertTrue((journal.directory / meta['snapshots'][0]['file']).exists())
            self.assertEqual(json.loads((journal.directory / 'messages.jsonl').read_text())['data']['data'], 3.)

    def test_rejected_review_keeps_original_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = RunRecorder(directory, {})
            journal.close()
            original = journal.directory
            journal.review(False)
            self.assertEqual(original, journal.directory)
            meta = json.loads(journal.metadata_path.read_text())
            self.assertEqual(meta['review_status'], 'rejected')
            self.assertIsNone(meta['record_name'])
            self.assertTrue((original / 'messages.jsonl').exists())

    def test_invalid_names_leave_run_unreviewed(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = RunRecorder(directory, {})
            journal.close()
            for name in ('', '.', '..', '../escape', 'a/b', 'a\\b', 'a\x00b', 'a\nb', '가' * 41):
                with self.subTest(name=name), self.assertRaises(ValueError):
                    journal.review(True, name)
            self.assertEqual(json.loads(journal.metadata_path.read_text())['review_status'], 'pending')

    def test_backpressure_is_bounded_and_reported(self):
        entered, release = threading.Event(), threading.Event()

        def blocked_conversion(message):
            entered.set()
            if not release.wait(3):
                raise RuntimeError('test writer was never released')
            return {'data': message.data}

        with tempfile.TemporaryDirectory() as directory:
            with patch('run_recorder.message_to_ordereddict', blocked_conversion):
                journal = RunRecorder(directory, {}, capacity=1)
                try:
                    self.assertTrue(journal.message('/fixture', 'std_msgs/msg/Float64', Float64(data=1.), 1))
                    self.assertTrue(entered.wait(2))
                    self.assertTrue(journal.message('/fixture', 'std_msgs/msg/Float64', Float64(data=2.), 2))
                    self.assertFalse(journal.message('/fixture', 'std_msgs/msg/Float64', Float64(data=3.), 3))
                finally:
                    release.set()
                    complete = journal.close()
                self.assertFalse(complete)
                meta = json.loads((journal.directory/'metadata.json').read_text())
                self.assertFalse(meta['complete'])
                self.assertEqual(meta['dropped'], 1)
                self.assertEqual(meta['written'], 2)
                rows = [json.loads(line) for line in (journal.directory/'messages.jsonl').read_text().splitlines()]
                self.assertEqual([row['data']['data'] for row in rows], [1., 2.])
                self.assertEqual([row['source_stamp_ns'] for row in rows], [None, None])


if __name__ == '__main__':
    unittest.main()
