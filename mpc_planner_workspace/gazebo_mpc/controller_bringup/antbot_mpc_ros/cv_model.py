"""World-frame GT state and constant-velocity prediction; no ROS dependencies."""
from dataclasses import dataclass
import math


def finite_vector(values, length):
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f'Expected {length} finite values')
    return result


@dataclass(frozen=True)
class WorldPlanarState:
    stamp_ns: int
    robot_id: str
    frame_id: str
    x: float
    y: float
    vx: float
    vy: float

    def __post_init__(self):
        if self.stamp_ns < 0 or not self.robot_id or not self.frame_id:
            raise ValueError('Invalid state timestamp or identity')
        finite_vector((self.x, self.y, self.vx, self.vy), 4)


def body_to_world_velocity(velocity_xyz, quaternion_xyzw):
    """Rotate a root-link velocity, including roll and pitch, into world axes."""
    vx, vy, vz = finite_vector(velocity_xyz, 3)
    qx, qy, qz, qw = finite_vector(quaternion_xyzw, 4)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-8 or abs(norm - 1.0) > 0.05:
        raise ValueError('Invalid odometry quaternion')
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    tx = 2.0 * (qy*vz - qz*vy)
    ty = 2.0 * (qz*vx - qx*vz)
    tz = 2.0 * (qx*vy - qy*vx)
    return (
        vx + qw*tx + qy*tz - qz*ty,
        vy + qw*ty + qz*tx - qx*tz,
        vz + qw*tz + qx*ty - qy*tx,
    )


def state_from_body_odometry(stamp_ns, robot_id, frame_id, position_xyz,
                             quaternion_xyzw, velocity_xyz):
    x, y, _ = finite_vector(position_xyz, 3)
    vx, vy, _ = body_to_world_velocity(velocity_xyz, quaternion_xyzw)
    return WorldPlanarState(stamp_ns, robot_id, frame_id, x, y, vx, vy)


def constant_velocity_prediction(state, start_ns, dt_ns, steps):
    """Return (stamp_ns, x, y); k=0 is start_ns, aligned from the GT sample.

    This is p[k] = p0 + v * dt * k, as in mpc_planner's CV helper, with
    p0 first propagated from the measurement time to the planning start.
    """
    if start_ns < state.stamp_ns or dt_ns <= 0 or steps < 2:
        raise ValueError('Invalid prediction start, interval, or step count')
    points = []
    for k in range(steps):
        stamp = start_ns + k*dt_ns
        elapsed = (stamp - state.stamp_ns) / 1e9
        points.append((stamp, state.x + state.vx*elapsed, state.y + state.vy*elapsed))
    return points


class PeerStateCache:
    """Reject replayed samples and invalidate cached state across clock resets."""

    def __init__(self):
        self.state = None
        self.received_wall_ns = None
        self.last_clock_ns = None

    def invalidate(self):
        self.state = None
        self.received_wall_ns = None

    def observe_clock(self, now_ns):
        reset = self.last_clock_ns is not None and now_ns < self.last_clock_ns
        self.last_clock_ns = now_ns
        if reset:
            self.invalidate()
        return reset

    def update(self, state, received_wall_ns):
        if self.state is not None and state.stamp_ns <= self.state.stamp_ns:
            return False
        self.state = state
        self.received_wall_ns = received_wall_ns
        return True

    def validity(self, now_ns, wall_ns, max_state_age_ns, max_receive_age_ns):
        if self.state is None:
            return 'waiting_for_peer'
        age = now_ns - self.state.stamp_ns
        if age < 0:
            return 'future_state'
        if age > max_state_age_ns:
            return 'stale_sim_time'
        if wall_ns - self.received_wall_ns > max_receive_age_ns:
            return 'stale_wall_time'
        return 'ok'
