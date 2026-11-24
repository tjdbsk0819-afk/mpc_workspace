"""Predict an observed object's motion without receiving its planned trajectory.

The planar Gaussian model matches mpc_planner: sigma(t) = noise*sqrt(dt*t).
All times are simulation seconds and all positions/velocities use world axes.
This module deliberately has no ROS, CasADi or NumPy dependency.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObjectState:
    robot_id: str
    stamp: float
    position: tuple
    velocity: tuple
    frame: str = "world"

    def __post_init__(self):
        if not self.robot_id or not self.frame or len(self.position) != 3 or len(self.velocity) != 3:
            raise ValueError("Object state requires an identity, frame, 3D position and velocity")
        if not all(math.isfinite(v) for v in (self.stamp, *self.position, *self.velocity)) or self.stamp < 0:
            raise ValueError("Object state must have finite values and a nonnegative timestamp")


@dataclass(frozen=True)
class PredictionConfig:
    model: str = "cv_gaussian"
    noise: float = 0.3
    risk: float = 0.05

    def __post_init__(self):
        if self.model not in ("cv", "cv_gaussian"):
            raise ValueError("prediction model must be cv or cv_gaussian")
        if not math.isfinite(self.noise) or self.noise <= 0 or not math.isfinite(self.risk) or not 0 < self.risk < 1:
            raise ValueError("Prediction noise must be positive and risk must be in (0, 1)")


@dataclass(frozen=True)
class ObjectPrediction:
    times: tuple
    positions: tuple  # rows are points: (N+1, 3)
    radii: tuple      # uncertainty padding; does not include robot size


def predict_object(state, start, dt, horizon, config=None):
    """Return N+1 points, with CV mean and Gaussian clearance padding by default.

    elapsed = planning_start - observation_stamp + k*dt
    mean = observed_position + observed_world_velocity*elapsed
    sigma = noise*sqrt(dt*elapsed)
    padding = sqrt(-2*log(risk))*sigma
    Robot size and nominal separation are added by the optimizer, not here.
    """
    config = config or PredictionConfig()
    if not math.isfinite(start) or start < state.stamp or not math.isfinite(dt) or dt <= 0:
        raise ValueError("Prediction requires start >= measurement time and dt > 0")
    if isinstance(horizon, bool) or not isinstance(horizon, int) or horizon < 1:
        raise ValueError("Prediction horizon must be a positive integer")
    times = tuple(start + k * dt for k in range(horizon + 1))
    positions = tuple(tuple(p + v * (t - state.stamp) for p, v in zip(state.position, state.velocity)) for t in times)
    # This is the upstream planar isotropic Gaussian confidence radius.
    radii = tuple(0.0 if config.model == "cv" else
                  math.sqrt(-2 * math.log(config.risk)) * config.noise * math.sqrt(dt * (t - state.stamp))
                  for t in times)
    return ObjectPrediction(times, positions, radii)


def observation_from_model_state(robot_id, values, model, stamp):
    """Adapt the repository's bicycle/drone/double-integrator state ordering."""
    x = tuple(float(v) for v in values)
    if model == "bicycle" and len(x) == 5:
        velocity = (x[4] * math.cos(x[3]), x[4] * math.sin(x[3]), 0.0)
    elif model == "drone" and len(x) == 6:
        velocity = x[3:6]
    elif model in ("dbl_int", "double_integrator") and len(x) == 12:
        velocity = x[6:9]
    else:
        raise ValueError(f"Unsupported state layout: model={model}, nx={len(x)}")
    return ObjectState(str(robot_id), stamp, x[:3], velocity)
