from dataclasses import asdict, dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class MPCConfig:
    dt: float = 0.2
    horizon: int = 20
    radius: float = 0.55
    margin: float = 0.15
    velocity: tuple = (0.65, 0.45, 0.8)
    acceleration: tuple = (0.8, 0.8, 1.2)
    q: tuple = (12., 12., 2.)
    r: tuple = (0.15, 0.15, 0.05)
    delta_r: tuple = (0.8, 0.8, 0.1)
    terminal_q: tuple = (24., 24., 4.)
    goal_tolerance: float = 0.12
    yaw_tolerance: float = 0.1
    waypoint_tolerance: float = 0.3
    stale_timeout: float = 0.6
    odom_timeout: float = 0.5
    solver_max_iterations: int = 150
    solver_max_seconds: float = 0.18
    max_time: float = 35.

    def __post_init__(self):
        import math
        if self.dt <= 0 or self.horizon < 2 or self.radius <= 0 or self.margin < 0:
            raise ValueError('Invalid MPC timestep, horizon, or collision geometry')
        for key in ['velocity', 'acceleration', 'q', 'r', 'delta_r', 'terminal_q']:
            values = getattr(self, key)
            if len(values) != 3 or not all(math.isfinite(x) and x > 0 for x in values):
                raise ValueError(f'{key} must have three positive finite values')
        if self.stale_timeout <= 0 or self.odom_timeout <= 0:
            raise ValueError('Timeouts must be positive')

    @property
    def separation(self):
        return 2*self.radius+self.margin

    @classmethod
    def load(cls, path=None):
        return cls(**json.loads(Path(path).read_text())) if path else cls()

    def save(self, path):
        Path(path).write_text(json.dumps(asdict(self), indent=2)+'\n')
