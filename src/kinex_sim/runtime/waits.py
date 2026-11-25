"""The world's pending wait, in simulation time. Called only by the physics owner, with no ROS imports."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WaitResult:
    success: bool
    message: str
    time_s: float
    done: tuple = ()  # whether each entry was over: the seconds, then the events
    timed_out: bool = False


def control_steps(seconds, period):
    """The whole control steps that cover seconds; inf beyond counting. A millionth of a step is none: a wait
    computed from the world's own time is due on its step, whatever floating point adds."""
    ticks = seconds / period if seconds > 0 else 0.
    return math.ceil(ticks - 1e-6) if ticks < math.inf else math.inf


class WaitScheduler:
    """One wait at a time: the world advances until its entries are over, all of them or the first."""

    def __init__(self):
        self._pending = None  # (first, entries, the step of its timeout, complete)

    @property
    def pending(self):
        return self._pending is not None

    def admit(self, first, seconds, events, timeout_s, time_s, step, period, ended, complete, interrupted=None):
        """Wait for entries: seconds of world time from now, and events, predicates such as a goal having finished.
        The wait ends once all are over, or with first once one is, and at a positive timeout_s whatever is left.
        ended: None while the episode runs, then why it ended, which every reply adds. interrupted: why a wait
        with anything left to wait for ends at once, as interrupt ends the pending one. A wait with nothing left,
        an observation barrier, still succeeds."""
        if self._pending:
            raise ValueError("another wait is pending; world waits are sequential")
        if not all(math.isfinite(value) and value >= 0 for value in (*seconds, timeout_s)):
            raise ValueError("seconds and timeout must be finite and nonnegative")
        if first and not (seconds or events):
            raise ValueError("wait_any needs an entry to wait for")
        ends = [step + control_steps(value, period) for value in seconds]
        if math.inf in ends:
            raise ValueError("seconds are too many to count in control steps")
        entries = [*(lambda now, end=end: now >= end for end in ends), *(lambda now, done=done: done() for done in events)]
        wait = first, entries, step + control_steps(timeout_s, period) if timeout_s else math.inf, complete
        done = tuple(entry(step) for entry in entries)
        if (any if first else all)(done):
            complete(WaitResult(True, f"ok; {ended}" if ended else "ok", time_s, done))
        elif ended:
            complete(WaitResult(False, f"episode is done; {ended}", time_s, done))
        elif interrupted:
            complete(WaitResult(False, interrupted, time_s, done))
        else:
            self._pending = wait

    def settle(self, time_s, step, ended=None):
        """ended: as for admit; the end completes the pending wait."""
        if not self._pending:
            return
        first, entries, limit, _ = self._pending
        met = (any if first else all)(entry(step) for entry in entries)
        if met or step >= limit:
            self.interrupt(time_s, step, f"ok; {ended}" if ended else "ok", True, not met)
        elif ended:
            self.interrupt(time_s, step, f"episode ended during wait; {ended}")

    def interrupt(self, time_s, step, message, success=False, timed_out=False):
        """End the pending wait now, with message."""
        if self._pending:
            (_, entries, _, complete), self._pending = self._pending, None
            complete(WaitResult(success, message, time_s, tuple(entry(step) for entry in entries), timed_out))
