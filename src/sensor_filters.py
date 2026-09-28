"""Sensor filters independent of SDK collection and robot state."""
import collections
import math
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# Filter Implementations
# ---------------------------------------------------------------------------

class MovingAverageFilter:
    """Moving average filter over a sliding window."""

    def __init__(self, window_size: int = 5):
        self.window_size = max(1, window_size)
        self.buffer = collections.deque(maxlen=self.window_size)

    def update(self, value: float) -> float:
        self.buffer.append(value)
        return sum(self.buffer) / len(self.buffer)

    def reset(self):
        self.buffer.clear()


class MedianFilter:
    """Median filter to reject impulsive sensor noise/spikes."""

    def __init__(self, window_size: int = 5):
        self.window_size = max(1, window_size)
        self.buffer = collections.deque(maxlen=self.window_size)

    def update(self, value: float) -> float:
        self.buffer.append(value)
        sorted_vals = sorted(self.buffer)
        n = len(sorted_vals)
        if n % 2 == 1:
            return sorted_vals[n // 2]
        else:
            return (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2.0

    def reset(self):
        self.buffer.clear()


class ExponentialMovingAverageFilter:
    """Exponential moving average (EMA / Low-pass filter)."""

    def __init__(self, alpha: float = 0.3):
        self.alpha = min(1.0, max(0.01, alpha))
        self.current_value: Optional[float] = None

    def update(self, value: float) -> float:
        if self.current_value is None:
            self.current_value = value
        else:
            self.current_value = self.alpha * value + (1.0 - self.alpha) * self.current_value
        return self.current_value

    def reset(self):
        self.current_value = None


class OutlierRejectionFilter:
    """Rejects out-of-bounds or physically impossible sensor jumps."""

    def __init__(self, min_valid: float, max_valid: float, max_rate_of_change: Optional[float] = None):
        self.min_valid = min_valid
        self.max_valid = max_valid
        self.max_rate_of_change = max_rate_of_change
        self.last_valid: Optional[float] = None

    def update(self, value: float) -> Tuple[float, bool]:
        if not (self.min_valid <= value <= self.max_valid):
            # Out of bounds
            return (self.last_valid if self.last_valid is not None else value, False)

        if self.max_rate_of_change is not None and self.last_valid is not None:
            if abs(value - self.last_valid) > self.max_rate_of_change:
                # Spike detected, reject or limit
                return (self.last_valid, False)

        self.last_valid = value
        return (value, True)

    def reset(self):
        self.last_valid = None


class SensorFilterPipeline:
    """Composite filter pipeline combining Outlier Rejection, Median, and EMA."""

    def __init__(
        self,
        min_valid: float = 0.0,
        max_valid: float = 1023.0,
        median_window: int = 5,
        ema_alpha: float = 0.35,
    ):
        self.outlier = OutlierRejectionFilter(min_valid=min_valid, max_valid=max_valid)
        self.median = MedianFilter(window_size=median_window)
        self.ema = ExponentialMovingAverageFilter(alpha=ema_alpha)

    def filter(self, raw_value: Optional[float]) -> Tuple[Optional[float], bool]:
        if raw_value is None or not math.isfinite(raw_value):
            return None, False

        checked_val, is_valid = self.outlier.update(raw_value)
        median_val = self.median.update(checked_val)
        filtered_val = self.ema.update(median_val)
        return filtered_val, is_valid

    def reset(self):
        self.outlier.reset()
        self.median.reset()
        self.ema.reset()


