"""Portable timed odometry references and a bounded Swerve tracking law."""
import bisect
import json
import math
from pathlib import Path


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def load_reference(path, scenario=None):
    data = json.loads(Path(path).read_text())
    if data.get('version') != 1 or data.get('frame') != 'world':
        raise ValueError('Unsupported reference format/frame')
    if scenario is not None and data['scenario'] != scenario:
        raise ValueError('Reference scenario does not match the scene')
    if set(data['robots']) != {f'robot_{r}' for r in 'abcd'}:
        raise ValueError('A four-robot reference is required')
    for robot, rows in data['robots'].items():
        if len(rows) < 2 or any(len(row) != 6 or not all(math.isfinite(v) for v in row) for row in rows):
            raise ValueError(f'Invalid samples: {robot}')
        if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
            raise ValueError(f'Non-increasing timestamps: {robot}')
    return data


def motion_start_time(rows):
    """Remove each recording's initial wait so all robot references start together."""
    return next((row[0] for row in rows if abs(row[4])>.03 or abs(row[5])>.03), rows[0][0])


class Trajectory:
    def __init__(self, rows):
        self.rows = rows
        self.times = [row[0] for row in rows]

    def at(self, elapsed):
        if elapsed >= self.times[-1]:
            return (*self.rows[-1][1:4], 0., 0.)
        if elapsed <= self.times[0]:
            return tuple(self.rows[0][1:])
        i = bisect.bisect_right(self.times, elapsed) - 1
        a, b = self.rows[i:i+2]
        f = (elapsed-a[0])/(b[0]-a[0])
        return (a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2]),
                wrap(a[3]+f*wrap(b[3]-a[3])), a[4]+f*(b[4]-a[4]), a[5]+f*(b[5]-a[5]))


    def world_velocity(self, elapsed):
        # Centered finite difference smooths odometry noise and preserves lateral motion.
        before, after = max(self.times[0], elapsed-.1), min(self.times[-1], elapsed+.1)
        if after <= before:
            return 0., 0.
        a, b = self.at(before), self.at(after)
        return (b[0]-a[0])/(after-before), (b[1]-a[1])/(after-before)


def tracking_command(state, reference, previous, dt, world_velocity=None):
    x, y, yaw = state[:3]
    rx, ry, ryaw, rv, rw = reference
    fx, fy = world_velocity if world_velocity is not None else (rv*math.cos(ryaw),rv*math.sin(ryaw))
    wx, wy = fx+1.5*(rx-x), fy+1.5*(ry-y)
    speed = math.hypot(wx,wy)
    if speed > 1.5:
        wx *= 1.5/speed; wy *= 1.5/speed
    vx = math.cos(yaw)*wx+math.sin(yaw)*wy
    vy = -math.sin(yaw)*wx+math.cos(yaw)*wy
    # Do not reverse, including when correcting a target behind the robot.
    vx = max(0.,vx)
    previous_vx = max(0.,previous[0])
    dx,dy = vx-previous_vx,vy-previous[1]
    change = math.hypot(dx,dy)
    scale = min(1., .7*dt/change) if change else 1.
    vx,vy = max(0.,previous_vx+scale*dx),previous[1]+scale*dy
    w = max(-.8,min(.8,rw+2.*wrap(ryaw-yaw)))
    w = previous[2]+max(-.5*dt,min(.5*dt,w-previous[2]))
    return vx,vy,w
