"""ROS-independent validation and local safety checks for the ROS adapter."""
import numpy as np
from .model import segment_distance, step


def fresh_stamp(stamp, now, received, wall_now, timeout):
    age = now-stamp
    return (all(np.isfinite(x) for x in (stamp, now, received, wall_now))
            and -1e-6 <= age <= timeout and 0 <= wall_now-received <= timeout)


def uniform_prediction_times(times, header_stamp):
    times = np.asarray(times, float)
    if len(times) < 2 or not np.isfinite(times).all() or abs(times[0]-header_stamp) > 1e-6:
        raise ValueError('Prediction must start at header stamp')
    intervals = np.diff(times)
    if intervals[0] <= 0 or not np.allclose(intervals, intervals[0], atol=1e-6, rtol=1e-6):
        raise ValueError('Prediction timestamps must be increasing and uniformly spaced')
    return float(intervals[0])


def local_command_is_safe(state, command, peer_states, inflation, cfg, obstacles=(), bounds=None):
    """Veto the first local command against the received peer plan, not an oracle."""
    own = [step(state, command, cfg.dt*k/10) for k in range(11)]
    peers = [(1-k/10)*peer_states[0]+k/10*peer_states[1] for k in range(11)]
    required = cfg.separation+max(inflation[:2])
    for k in range(10):
        if segment_distance(own[k], own[k+1], peers[k], peers[k+1]) < required-1e-5:
            return False
        for ox, oy, radius in obstacles:
            if segment_distance(own[k], own[k+1], [ox, oy], [ox, oy]) < cfg.radius+cfg.margin+radius-1e-5:
                return False
        if bounds:
            xmin, xmax, ymin, ymax = bounds
            if not (xmin+cfg.radius <= own[k+1][0] <= xmax-cfg.radius and ymin+cfg.radius <= own[k+1][1] <= ymax-cfg.radius):
                return False
    return True
