"""고규범 신규 로직 (2026-10-21): ROS-independent stop observation and interpolation."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    received_s: float
    speed_mps: float
    speed_valid: bool
    x_m: float = 0.0
    y_m: float = 0.0
    position_valid: bool = False


class StopWindow:
    """Only distinct, advancing source samples can establish a continuous stop."""
    def __init__(self, speed_mps=0.1, hold_s=0.5, timeout_s=0.3):
        if any(not math.isfinite(v) or v <= 0 for v in (speed_mps, hold_s, timeout_s)):
            raise ValueError('stop thresholds and timeout must be finite and positive')
        self.threshold = speed_mps
        self.hold_ns = round(hold_s * 1e9)
        self.timeout_s = timeout_s
        self.last = self.start = self.confirmed = None
        self.admitted = False

    def reset(self):
        self.start = self.confirmed = None

    def fresh(self, now_ns, now_s):
        return bool(self.admitted and self.last is not None and self.last.speed_valid and
                0 <= now_s - self.last.received_s <= self.timeout_s and
                0 <= now_ns - self.last.stamp_ns <= self.timeout_s * 1e9)

    def expire(self, now_ns, now_s):
        if not self.fresh(now_ns, now_s):
            self.reset()

    def observe(self, sample, now_ns):
        if (sample.stamp_ns <= 0 or sample.stamp_ns > now_ns or
                now_ns - sample.stamp_ns > self.timeout_s * 1e9):
            self.admitted = False
            self.reset()
            return False
        if self.last is not None:
            if sample.stamp_ns <= self.last.stamp_ns:
                # Repeated samples do not renew validity; regression breaks continuity.
                if sample.stamp_ns < self.last.stamp_ns:
                    self.admitted = False
                    self.reset()
                if (not sample.speed_valid or not math.isfinite(sample.speed_mps) or
                        abs(sample.speed_mps) > self.threshold):
                    self.admitted = False
                    self.reset()
                return False
            if (sample.stamp_ns - self.last.stamp_ns > self.timeout_s * 1e9 or
                    sample.received_s - self.last.received_s > self.timeout_s):
                self.reset()
        self.last = sample
        self.admitted = bool(sample.speed_valid and math.isfinite(sample.speed_mps))
        if not sample.speed_valid or not math.isfinite(sample.speed_mps) or abs(sample.speed_mps) > self.threshold:
            self.reset()
            return True
        if self.start is None:
            self.start = sample
        if self.confirmed is None and sample.stamp_ns - self.start.stamp_ns >= self.hold_ns:
            self.confirmed = sample
        return True


class StopObservation:
    """One resolved command topic per instance; no command transmission or control policy."""
    def __init__(self, speed_mps=0.1, hold_s=0.5, timeout_s=0.3):
        self.window = StopWindow(speed_mps, hold_s, timeout_s)
        self.samples = deque(maxlen=1000)
        self.active = False
        self.stop_id = 0
        self.command_ns = self.transmit_ns = 0
        self.last_event_s = None
        self.last_stage_ns = {}
        self.episode_ns = 0
        self.release_ns = 0
        self.command_position = self.transmit_position = None

    def command(self, stamp_ns, stage, stopping, received_s, now_ns):
        if stage not in (1, 2, 3) or not 0 < stamp_ns <= now_ns:
            return None
        if now_ns - stamp_ns > self.window.timeout_s * 1e9:
            return None
        if stamp_ns <= self.last_stage_ns.get(stage, 0):
            return None
        self.last_stage_ns[stage] = stamp_ns
        # DDS can deliver a prior stage after a later one. Do not re-open old episodes.
        if stamp_ns < self.release_ns or (not stopping and stamp_ns < self.episode_ns):
            return None
        self.last_event_s = received_s
        transition = None
        if stopping and not self.active:
            self.stop_id += 1
            self.active = True
            self.command_ns = self.transmit_ns = 0
            self.command_position = self.transmit_position = None
            self.episode_ns = stamp_ns
            self.window.reset()
            transition = 'stop_command'
        elif not stopping:
            if self.active:
                transition = 'stop_released'
            self.active = False
            self.episode_ns = max(self.episode_ns, stamp_ns)
            self.release_ns = max(self.release_ns, stamp_ns)
        if stopping:
            if stage == 1 and self.command_ns == 0:
                self.command_ns = stamp_ns
            elif stage == 2 and self.transmit_ns == 0:
                self.transmit_ns = stamp_ns
                transition = 'stop_transmitted'
            # DRY_RUN never establishes a physical transmit timestamp.
        self._positions()
        return transition

    def observe(self, sample, now_ns):
        if self.active and sample.stamp_ns < self.episode_ns:
            return False
        accepted = self.window.observe(sample, now_ns)
        if accepted:
            self.samples.append(sample)
            self._positions()
        return accepted

    def interpolate(self, stamp_ns):
        if not stamp_ns:
            return None
        previous = None
        for sample in self.samples:
            if sample.stamp_ns == stamp_ns:
                return (sample.x_m, sample.y_m) if sample.position_valid else None
            if sample.stamp_ns > stamp_ns:
                if (previous is None or not previous.position_valid or not sample.position_valid or
                        sample.stamp_ns - previous.stamp_ns > self.window.timeout_s * 1e9):
                    return None
                fraction = (stamp_ns - previous.stamp_ns) / (sample.stamp_ns - previous.stamp_ns)
                return (previous.x_m + fraction * (sample.x_m - previous.x_m),
                        previous.y_m + fraction * (sample.y_m - previous.y_m))
            previous = sample
        return None  # No extrapolation or made-up zero coordinates.

    def _positions(self):
        if self.active:
            if self.command_position is None:
                self.command_position = self.interpolate(self.command_ns)
            if self.transmit_position is None:
                self.transmit_position = self.interpolate(self.transmit_ns)

    def snapshot(self, now_ns, now_s):
        self.window.expire(now_ns, now_s)
        last, start, confirmed = self.window.last, self.window.start, self.window.confirmed
        valid = self.window.fresh(now_ns, now_s)
        stopped = valid and confirmed is not None
        stop_position = (start.x_m, start.y_m) if stopped and start.position_valid else None
        distance_valid = self.active and self.transmit_position is not None and stop_position is not None
        return dict(stop_id=self.stop_id, command_active=self.active,
                    command_fresh=self.last_event_s is not None and
                    0 <= now_s - self.last_event_s <= self.window.timeout_s,
                    speed_valid=valid, speed_mps=last.speed_mps if valid else 0.0,
                    sample_ns=last.stamp_ns if last else 0, stopped=stopped,
                    stopped_since_ns=start.stamp_ns if stopped else 0,
                    confirmed_at_ns=confirmed.stamp_ns if stopped else 0,
                    command_ns=self.command_ns, transmit_ns=self.transmit_ns,
                    command_position=self.command_position, transmit_position=self.transmit_position,
                    stop_position=stop_position, distance_valid=distance_valid,
                    distance_m=math.dist(self.transmit_position, stop_position) if distance_valid else 0.0,
                    reason='STOPPED' if stopped else ('MOVING_OR_SETTLING' if valid else 'GNSS_UNAVAILABLE'))
