"""Pure geometry for the final RPP command check (not an MPC replacement)."""
import math


def minimum_cv_clearance(x, y, yaw, speed, yaw_rate, peer_x, peer_y,
                         peer_vx, peer_vy, horizon, segments=20):
    """Circle-center distance: own constant-turn motion versus peer CV.

    Minimize distance on each relative line segment, rather than checking only
    endpoints. Used as a conservative short-horizon command veto.
    """
    if not all(math.isfinite(v) for v in (x, y, yaw, speed, yaw_rate,
               peer_x, peer_y, peer_vx, peer_vy, horizon)) or horizon <= 0:
        raise ValueError('Nonfinite motion or invalid guard horizon')
    previous = (x-peer_x, y-peer_y)
    closest = math.hypot(*previous)
    for k in range(1, segments+1):
        t = horizon*k/segments
        if abs(yaw_rate) < 1e-6:
            px, py = x+speed*t*math.cos(yaw), y+speed*t*math.sin(yaw)
        else:
            px = x+speed/yaw_rate*(math.sin(yaw+yaw_rate*t)-math.sin(yaw))
            py = y-speed/yaw_rate*(math.cos(yaw+yaw_rate*t)-math.cos(yaw))
        current = (px-peer_x-peer_vx*t, py-peer_y-peer_vy*t)
        dx, dy = current[0]-previous[0], current[1]-previous[1]
        norm = dx*dx+dy*dy
        fraction = 0. if norm < 1e-15 else max(0., min(1., -(previous[0]*dx+previous[1]*dy)/norm))
        closest = min(closest, math.hypot(previous[0]+fraction*dx, previous[1]+fraction*dy))
        previous = current
    return closest
