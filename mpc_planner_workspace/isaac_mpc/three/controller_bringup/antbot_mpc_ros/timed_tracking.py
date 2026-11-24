"""Time-aligned sampling and bounded unicycle execution shared by tracking/guard."""
from bisect import bisect_right
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def approach(value, target, amount):
    return value + max(-amount, min(amount, target-value))


@dataclass
class ProximityReleaseGate:
    """Latch a rejected command until consecutive safe simulation-time checks.

    No command is made safe by this gate: callers must rerun the full clearance
    check first. Other holds interrupt the release window without clearing the
    stop latch. All timestamps and durations are integer nanoseconds.
    """
    duration_ns: int
    maximum_gap_ns: int
    blocked: bool = False
    safe_since: int | None = None
    previous_sample: int | None = None

    def interrupt(self):
        self.safe_since = self.previous_sample = None

    def stop(self):
        self.blocked = True
        self.interrupt()

    def allow(self, now):
        if not self.blocked:
            return True
        if (self.previous_sample is None or now < self.previous_sample or
                now-self.previous_sample > self.maximum_gap_ns):
            self.safe_since = now
        self.previous_sample = now
        if now-self.safe_since < self.duration_ns:
            return False
        self.blocked = False
        self.interrupt()
        return True


@dataclass(frozen=True)
class Motion:
    x: float
    y: float
    yaw: float
    v: float
    w: float


def step(state, target_v, target_w, dt, acceleration, yaw_acceleration):
    """Velocity targets, bounded actuator response, and midpoint unicycle motion."""
    v = approach(state.v, target_v, acceleration*dt)
    w = approach(state.w, target_w, yaw_acceleration*dt)
    vm, wm = (state.v+v)/2., (state.w+w)/2.
    return Motion(state.x+vm*math.cos(state.yaw+wm*dt/2.)*dt,
                  state.y+vm*math.sin(state.yaw+wm*dt/2.)*dt,
                  state.yaw+wm*dt, v, w)


@dataclass(frozen=True)
class TimedTrajectory:
    # seconds relative to plan header; states and controls from one solver output
    times: tuple
    poses: tuple
    velocities: tuple
    accelerations: tuple
    yaw_rates: tuple

    def __post_init__(self):
        n = len(self.times)
        if n < 2 or any(len(v) != n for v in (self.poses,self.velocities,self.accelerations,self.yaw_rates)):
            raise ValueError('Missing MPC motion profile')
        if abs(self.times[0]) > 1e-9 or any(b<=a for a,b in zip(self.times,self.times[1:])):
            raise ValueError('Invalid MPC motion time grid')
        values = (*self.times,*self.velocities,*self.accelerations,*self.yaw_rates,
                  *(c for pose in self.poses for c in pose))
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Nonfinite MPC motion profile')

    def sample(self, seconds):
        if seconds < -1e-6 or seconds > self.times[-1]+1e-6:
            raise ValueError('MPC motion profile expired')
        t = max(0., min(seconds,self.times[-1]))
        k = min(len(self.times)-1, max(0,bisect_right(self.times,t)-1))
        x,y,yaw = self.poses[k]
        tau = t-self.times[k]
        v,a,w = self.velocities[k],self.accelerations[k],self.yaw_rates[k]
        # RK4 of the SAME constant-a, constant-w stage used by the MPC model.
        midv,endv = v+.5*a*tau,v+a*tau
        x += tau/6.*(v*math.cos(yaw)+4*midv*math.cos(yaw+.5*w*tau)+endv*math.cos(yaw+w*tau))
        y += tau/6.*(v*math.sin(yaw)+4*midv*math.sin(yaw+.5*w*tau)+endv*math.sin(yaw+w*tau))
        return Motion(x,y,yaw+w*tau,endv,w)

    def window(self, age, horizon, period=.025):
        end = min(age+horizon,self.times[-1])
        if end <= age:
            raise ValueError('Empty MPC tracking window')
        count = max(1,math.ceil((end-age)/period))
        return [self.sample(age+(end-age)*k/count) for k in range(count+1)]


def command_clearance(state, target_v, target_w, peer, hold_time, acceleration,
                      yaw_acceleration, braking_deceleration, integration_dt=.005):
    """Predict the actual bounded command, then a braking fallback; peer is CV.

    Everything starts at the SAME execution epoch. A stop command must still
    account for the measured velocity's finite braking distance.
    """
    px,py,pvx,pvy = peer
    horizon = hold_time+max(abs(state.v),abs(target_v))/braking_deceleration
    horizon += max(abs(state.w),abs(target_w))/yaw_acceleration
    count = max(1,math.ceil(horizon/integration_dt))
    dt = horizon/count
    closest = math.hypot(state.x-px,state.y-py)
    previous = (state.x-px,state.y-py)
    current = state
    for k in range(1,count+1):
        t = k*dt
        holding = (k-1)*dt < hold_time
        current = step(current,target_v if holding else 0.,target_w if holding else 0.,dt,
                       acceleration if holding else braking_deceleration,yaw_acceleration)
        relative = (current.x-px-pvx*t,current.y-py-pvy*t)
        dx,dy = relative[0]-previous[0],relative[1]-previous[1]
        norm = dx*dx+dy*dy
        f = 0. if norm < 1e-15 else max(0.,min(1.,-(previous[0]*dx+previous[1]*dy)/norm))
        closest = min(closest,math.hypot(previous[0]+f*dx,previous[1]+f*dy))
        previous = relative
    return closest


def filter_command(state, desired_v, desired_w, peer, separation, hold_time,
                   acceleration, yaw_acceleration, braking_deceleration, *,
                   mpc_yaw_rate=None, rpp_yaw_rate=None):
    """Slow a translating RPP command without changing its requested curvature.

    Prefer translating candidates. If all fail, retain the bounded RPP turn
    only when its direction agrees with the current MPC and raw RPP request.
    Every candidate, including rotation and braking stop, starts from measured
    motion and must pass the same clearance check.
    """
    return filter_command_multi(state,desired_v,desired_w,[(peer,separation)],hold_time,
                                acceleration,yaw_acceleration,braking_deceleration,
                                mpc_yaw_rate=mpc_yaw_rate,rpp_yaw_rate=rpp_yaw_rate)


def filter_command_multi(state, desired_v, desired_w, neighbors, hold_time,
                         acceleration, yaw_acceleration, braking_deceleration, *,
                         mpc_yaw_rate=None, rpp_yaw_rate=None):
    """Select one common command that passes every neighbor's clearance bound."""
    if not neighbors:
        raise ValueError('At least one neighbor is required')
    def checked(v,w):
        closest=math.inf
        for peer,separation in neighbors:
            distance=command_clearance(state,v,w,peer,hold_time,acceleration,
                                       yaw_acceleration,braking_deceleration)
            if distance < separation:
                return None
            closest=min(closest,distance)
        return closest
    moving = abs(desired_v) > 1e-4
    for scale in (1., .75, .5, .25, .125):
        v = desired_v*scale
        w = desired_w if abs(desired_v) <= 1e-4 else desired_w*scale
        clearance = checked(v,w)
        if clearance is not None:
            return v,w,clearance,scale
    turn_authorized = moving and abs(desired_w) > 1e-4 and all(
        rate is not None and math.isfinite(rate) and desired_w*rate > 0.
        for rate in (mpc_yaw_rate, rpp_yaw_rate))
    if turn_authorized:
        clearance = checked(0.,desired_w)
        if clearance is not None:
            return 0.,desired_w,clearance,0.
    clearance = checked(0.,0.)
    if clearance is not None:
        return 0.,0.,clearance,0.
    return None
