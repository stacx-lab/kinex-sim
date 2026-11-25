"""The world's pending wait, in simulation time. Called only by the physics owner, with no ROS imports."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WaitResult:
    success: bool
    message: str
    time_s: float


def control_steps(seconds, period):
    """The whole control steps that cover seconds; inf beyond counting. A millionth of a step is none: a wait
    computed from the world's own time is due on its step, whatever floating point adds."""
    ticks = seconds / period if seconds > 0 else 0.
    return math.ceil(ticks - 1e-6) if ticks < math.inf else math.inf


class WaitScheduler:
    """One wait at a time: the world advances until its time is over, or something ends it sooner."""

    def __init__(self):
        self._pending = None  # (the step its time is over at, complete)

    @property
    def pending(self):
        return self._pending is not None

    def admit(self, seconds, time_s, step, period, ended, complete, interrupted=None):
        """Wait seconds of world time from now; infinite seconds wait until interrupt ends the wait.
        ended: None while the episode runs, then why it ended, which every reply adds. interrupted: (success,
        message) with which a wait that has time left ends at once, as interrupt ends the pending one. A wait
        with no time left, an observation barrier, still succeeds."""
        if self._pending:
            raise ValueError("another wait is pending; world waits are sequential")
        if math.isnan(seconds) or seconds < 0:
            raise ValueError("seconds must be nonnegative")
        end = step + control_steps(seconds, period)
        if step >= end:
            complete(WaitResult(True, f"ok; {ended}" if ended else "ok", time_s))
        elif ended:
            complete(WaitResult(False, f"episode is done; {ended}", time_s))
        elif interrupted:
            complete(WaitResult(*interrupted, time_s))
        else:
            self._pending = end, complete

    def settle(self, time_s, step, ended=None):
        """ended: as for admit; the end completes the pending wait."""
        if not self._pending:
            return
        if step >= self._pending[0]:
            self.interrupt(time_s, f"ok; {ended}" if ended else "ok", True)
        elif ended:
            self.interrupt(time_s, f"episode ended during wait; {ended}")

    def interrupt(self, time_s, message, success=False):
        """End the pending wait now, with message."""
        if self._pending:
            (_, complete), self._pending = self._pending, None
            complete(WaitResult(success, message, time_s))
