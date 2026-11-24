"""Timestamped prediction contract for simulated and ROS communication."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Prediction:
    stamp: float
    dt: float
    states: np.ndarray  # (N+1, 3), common world frame
    frame: str = 'world'

    def __post_init__(self):
        self.states = np.asarray(self.states, float)
        if (not np.isfinite(self.stamp) or not np.isfinite(self.dt) or self.dt <= 0
                or self.states.ndim != 2 or self.states.shape[1] != 3
                or len(self.states) < 2 or not np.isfinite(self.states).all()):
            raise ValueError('Malformed prediction')

    def align(self, now, config, frame='world'):
        """Interpolate at absolute times; endpoint holds require uncertainty inflation."""
        age = now-self.stamp
        if self.frame != frame or age < -1e-6 or age > config.stale_timeout:
            raise ValueError('Wrong frame, future, or stale prediction')
        source = self.stamp+np.arange(len(self.states))*self.dt
        target = now+np.arange(config.horizon+1)*config.dt
        aligned = np.column_stack([np.interp(target, source, self.states[:, j]) for j in range(3)])
        speed = float(np.linalg.norm(config.velocity[:2]))
        # The initial age also bounds unmodelled motion while the plan was in transit.
        inflation = speed*(age+np.maximum(0, target-source[-1]))
        return aligned, inflation


def hold_prediction(state, now, config, frame='world'):
    return Prediction(now, config.dt, np.tile(state, (config.horizon+1, 1)), frame)
