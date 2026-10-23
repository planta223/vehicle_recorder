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
