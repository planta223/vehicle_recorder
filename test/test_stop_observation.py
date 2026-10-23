#!/usr/bin/env python3
"""고규범 신규 검증 (2026-10-21): timing, continuity, clock domains and interpolation."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from stop_observation import Sample, StopObservation, StopWindow


def sample(t, speed=0.0, valid=True, position=True):
    return Sample(round(t * 1e9), t, speed, valid, 2 * t, t, position)


class StopTests(unittest.TestCase):
    def observe(self, window, t, speed=0.0, valid=True):
        return window.observe(sample(t, speed, valid), round(t * 1e9))

    def test_confirm_only_after_half_second_and_keep_first_stamp(self):
        w = StopWindow()
        for t in (1.0, 1.1, 1.2, 1.3, 1.4):
            self.observe(w, t, -0.1)
            self.assertIsNone(w.confirmed)
        self.observe(w, 1.5, 0.1)
        self.assertEqual(w.start.stamp_ns, 1_000_000_000)
        self.assertEqual(w.confirmed.stamp_ns, 1_500_000_000)

    def test_cached_or_duplicate_sample_never_confirms(self):
        w = StopWindow()
        self.observe(w, 1.0)
        for t in (1.1, 1.2, 1.3, 1.4, 1.5):
            w.observe(sample(1.0), round(t * 1e9))
            w.expire(round(t * 1e9), t)
        self.assertIsNone(w.confirmed)
        self.assertFalse(w.fresh(1_500_000_000, 1.5))

    def test_speed_invalid_nan_and_gap_reset_window(self):
        for bad in ('speed', 'invalid', 'nan', 'gap', 'future', 'regression'):
            w = StopWindow()
            self.observe(w, 1.0)
            self.observe(w, 1.1)
            if bad == 'gap': self.observe(w, 1.5)
            elif bad == 'future': w.observe(sample(1.3), 1_200_000_000)
            elif bad == 'regression': self.observe(w, 1.05)
            else: self.observe(w, 1.2, 0.11 if bad == 'speed' else float('nan') if bad == 'nan' else 0,
                               valid=bad != 'invalid')
            self.observe(w, 1.6)
            self.assertIsNone(w.confirmed, bad)

    def test_missing_gnss_revokes_confirmed_stop(self):
        o = StopObservation()
        for t in (1.0, 1.2, 1.4, 1.6): o.observe(sample(t), round(t * 1e9))
        self.assertTrue(o.snapshot(1_600_000_000, 1.6)['stopped'])
        self.assertFalse(o.snapshot(2_000_000_000, 2.0)['stopped'])

    def test_first_command_and_send_times_interpolation_and_distance(self):
        o = StopObservation()
        o.observe(sample(1.0, 2), 1_000_000_000)
        self.assertEqual(o.command(1_050_000_000, 1, True, 1.05, 1_050_000_000), 'stop_command')
        o.command(1_060_000_000, 2, True, 1.06, 1_060_000_000)
        for t in (1.1, 1.2, 1.3, 1.4, 1.5, 1.6):
            o.command(round(t * 1e9), 1, True, t, round(t * 1e9))
            o.observe(sample(t), round(t * 1e9))
        d = o.snapshot(1_600_000_000, 1.6)
        self.assertEqual(d['command_ns'], 1_050_000_000)
        self.assertEqual(d['transmit_ns'], 1_060_000_000)
        self.assertEqual(d['stopped_since_ns'], 1_100_000_000)
        self.assertEqual(d['confirmed_at_ns'], 1_600_000_000)
        self.assertEqual(d['stop_id'], 1)
        self.assertAlmostEqual(d['command_position'][0], 2.1)
        self.assertTrue(d['distance_valid'])
        self.assertAlmostEqual(d['distance_m'], (0.08 ** 2 + 0.04 ** 2) ** 0.5)

    def test_dry_run_and_no_position_never_fabricate_distance(self):
        o = StopObservation()
        o.command(1_000_000_000, 3, True, 1, 1_000_000_000)
        for t in (1.0, 1.2, 1.4, 1.6):
            o.observe(sample(t, position=False), round(t * 1e9))
        d = o.snapshot(1_600_000_000, 1.6)
        self.assertTrue(d['stopped'])
        self.assertEqual(d['transmit_ns'], 0)
        self.assertFalse(d['distance_valid'])

    def test_delayed_command_can_follow_send_but_old_stop_cannot_follow_release(self):
        o = StopObservation()
        o.command(1_060_000_000, 2, True, 1.06, 1_070_000_000)
        o.command(1_050_000_000, 1, True, 1.07, 1_070_000_000)
        self.assertEqual(o.command_ns, 1_050_000_000)
        self.assertEqual(o.stop_id, 1)
        o.command(2_000_000_000, 1, False, 2, 2_000_000_000)
        o.command(1_990_000_000, 2, True, 2.01, 2_010_000_000)
        self.assertFalse(o.active)
        o.command(3_000_000_000, 1, True, 3, 3_000_000_000)
        self.assertEqual(o.stop_id, 2)


if __name__ == '__main__':
    unittest.main()
