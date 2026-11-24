"""A single-robot nonlinear MPC; peers enter only as object parameters.

IPOPT is retained from dmpc. Isaac's planar AntBot adapter uses the same
second-order unicycle and timestamped motion profile as mpc_planner's RPP.
"""
from dataclasses import dataclass
import math
import time

import casadi as ca
import numpy as np

from .prediction import ObjectState, PredictionConfig, predict_object


@dataclass(frozen=True)
class PlannerConfig:
    horizon: int = 30
    dt: float = 0.2
    allow_reverse: bool = False
    max_speed: float = 0.5
    max_acceleration: float = 0.5
    max_yaw_rate: float = 0.8
    max_yaw_acceleration: float = 0.5
    command_dt: float = 0.1
    separation: float = 1.15
    tracking_margin: float = 0.05
    current_clearance_check_enabled: bool = True
    solve_seconds: float = 0.08
    mu_strategy: str = "monotone"
    goal_tolerance: float = 0.2
    goal_braking_deceleration: float = 0.25
    goal_reaction_time: float = 0.3
    goal_stop_reserve: float = 0.08
    prediction_model: str = "cv_gaussian"
    prediction_noise: float = 0.3
    prediction_risk: float = 0.05
    bounds: tuple | None = None
    obstacles: tuple = ()
    wall_inset: float = 0.55
    wall_planning_margin: float = 0.001
    static_padding: float = 0.7
    peer_count: int = 1

    def __post_init__(self):
        if isinstance(self.peer_count, bool) or not isinstance(self.peer_count, int) or not 1 <= self.peer_count <= 3:
            raise ValueError("peer_count must be an integer from 1 to 3")
        if isinstance(self.horizon, bool) or not isinstance(self.horizon, int) or not 3 <= self.horizon <= 1000:
            raise ValueError("Horizon must be an integer in [3, 1000]")
        for key in ("dt", "max_speed", "max_acceleration", "max_yaw_rate", "max_yaw_acceleration",
                    "command_dt", "separation", "goal_tolerance", "wall_inset", "static_padding",
                    "goal_braking_deceleration", "goal_reaction_time", "goal_stop_reserve"):
            if not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if not math.isfinite(self.solve_seconds) or self.solve_seconds < 0:
            raise ValueError("solve_seconds must be nonnegative; zero disables time limits")
        if self.goal_braking_deceleration > self.max_acceleration:
            raise ValueError('Goal braking deceleration must not exceed model acceleration')
        if self.goal_stop_reserve >= self.goal_tolerance:
            raise ValueError('Goal stop reserve must lie inside the arrival tolerance')
        for key in ('tracking_margin', 'wall_planning_margin'):
            if not math.isfinite(getattr(self, key)) or getattr(self, key) < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        if self.mu_strategy not in ("monotone", "adaptive"):
            raise ValueError("Unsupported IPOPT barrier strategy")
        PredictionConfig(self.prediction_model, self.prediction_noise, self.prediction_risk)
        if self.bounds is not None:
            if len(self.bounds) != 4 or not all(math.isfinite(v) for v in self.bounds):
                raise ValueError("Bounds must be xmin, xmax, ymin, ymax")
            xmin, xmax, ymin, ymax = self.bounds
            if xmax - xmin <= 2*self.planning_wall_inset or ymax-ymin <= 2*self.planning_wall_inset:
                raise ValueError("Bounds leave no space for the robot")
        for obstacle in self.obstacles:
            if len(obstacle) != 3 or not all(math.isfinite(v) for v in obstacle) or obstacle[2] <= 0:
                raise ValueError("Obstacles must be finite x, y, positive radius")

    @property
    def planning_wall_inset(self):
        # Leave room for the executor's conservative inter-sample curve guard.
        return self.wall_inset+self.wall_planning_margin


@dataclass(frozen=True)
class Plan:
    stamp: float
    states: np.ndarray       # x, y, yaw, forward speed; shape (4, N+1)
    controls: np.ndarray     # acceleration, yaw rate; shape (2, N)
    prediction: object
    elapsed: float
    objective: float
    goal: np.ndarray
    predictions: tuple = ()
    peer_distances: tuple = ()


def integrate(state, control, dt):
    """Simpson/RK4 unicycle step, matching RPP's inter-knot sampling."""
    x, y, yaw, speed = state[0], state[1], state[2], state[3]
    acceleration, rate = control[0], control[1]
    half = yaw + rate*dt/2
    end = yaw + rate*dt
    middle_speed = speed + acceleration*dt/2
    next_speed = speed + acceleration*dt
    return ca.vertcat(
        x + dt/6*(speed*ca.cos(yaw) + 4*middle_speed*ca.cos(half) + next_speed*ca.cos(end)),
        y + dt/6*(speed*ca.sin(yaw) + 4*middle_speed*ca.sin(half) + next_speed*ca.sin(end)),
        end, next_speed)


def goal_speed_limit(distance, config):
    """Speed whose conservative stopping distance fits before the goal reserve."""
    a = config.goal_braking_deceleration
    lag_speed = a*config.goal_reaction_time
    return min(config.max_speed,
               math.sqrt(lag_speed**2+2*a*max(0., distance-config.goal_stop_reserve))-lag_speed)


def yaw_command_reference(measured, command, age, timeout, maximum):
    """Input slew limits refer to the issued command, not actuator response."""
    if (command is not None and math.isfinite(command) and abs(command) <= maximum+1e-3
            and math.isfinite(age) and 0 <= age <= timeout):
        return command
    return measured


def workspace_interval_safe(state, control, duration, bounds, inset):
    """Match the native Bounds::interval_safe check before publishing a plan."""
    xmin, xmax, ymin, ymax = bounds
    def clearance(x, y):
        return min(x-xmin-inset, xmax-inset-x, y-ymin-inset, ymax-inset-y)
    x, y, yaw, speed = map(float, state)
    acceleration, rate = map(float, control)
    if clearance(x, y) < -1e-7:
        return False
    count = max(1, math.ceil(duration/.01))
    dt = duration/count
    for _ in range(count):
        vm, ve = speed+.5*acceleration*dt, speed+acceleration*dt
        nx = x+dt/6*(speed*math.cos(yaw)+4*vm*math.cos(yaw+.5*rate*dt)+ve*math.cos(yaw+rate*dt))
        ny = y+dt/6*(speed*math.sin(yaw)+4*vm*math.sin(yaw+.5*rate*dt)+ve*math.sin(yaw+rate*dt))
        bend = (abs(acceleration)+max(abs(speed), abs(ve))*abs(rate))*dt*dt/8
        if min(clearance(x, y), clearance(nx, ny))+1e-7 < bend:
            return False
        x, y, yaw, speed = nx, ny, yaw+rate*dt, ve
    return True


class LocalObjectMPC:
    def __init__(self, config=None):
        self.config = c = config or PlannerConfig()
        self.prediction_config = PredictionConfig(c.prediction_model, c.prediction_noise, c.prediction_risk)
        self.previous = None
        self.opti = o = ca.Opti()
        self.X, self.U = o.variable(4, c.horizon+1), o.variable(2, c.horizon)
        self.initial = o.parameter(4)
        self.goal = o.parameter(3)
        self.reference = o.parameter(2, c.horizon+1)
        self.peers = [o.parameter(2, c.horizon+1) for _ in range(c.peer_count)]
        self.distances = [o.parameter(1, c.horizon+1) for _ in range(c.peer_count)]
        self.peer, self.distance = self.peers[0], self.distances[0]
        self.previous_rate = o.parameter()
        o.subject_to(self.X[:, 0] == self.initial)
        o.subject_to(o.bounded(-c.max_speed if c.allow_reverse else 0, self.X[3, :], c.max_speed))
        o.subject_to(o.bounded(-c.max_acceleration, self.U[0, :], c.max_acceleration))
        o.subject_to(o.bounded(-c.max_yaw_rate, self.U[1, :], c.max_yaw_rate))
        o.subject_to(o.bounded(-c.max_yaw_acceleration*c.command_dt,
                               self.U[1, 0]-self.previous_rate, c.max_yaw_acceleration*c.command_dt))
        objective = 0
        for k in range(c.horizon):
            x, u = self.X[:, k], self.U[:, k]
            o.subject_to(self.X[:, k+1] == integrate(x, u, c.dt))
            next_state = self.X[:, k+1]
            remaining_next = ca.sqrt(ca.sumsqr(next_state[:2]-self.goal[:2])+1e-8)
            lag_speed = c.goal_braking_deceleration*c.goal_reaction_time
            braking_limit = ca.sqrt(lag_speed**2+2*c.goal_braking_deceleration*
                                   ca.fmax(0., remaining_next-c.goal_stop_reserve))-lag_speed
            # Receding plans start from measured speed. Permit the unavoidable
            # maximum-braking transient when that speed is already outside the
            # envelope, rather than making the OCP infeasible on initialization.
            braking_transient = (ca.fabs(self.initial[3]) if c.allow_reverse else self.initial[3])-c.max_acceleration*(k+1)*c.dt
            o.subject_to((ca.fabs(next_state[3]) if c.allow_reverse else next_state[3]) <= ca.fmax(braking_limit, braking_transient))
            if k:
                o.subject_to(o.bounded(-c.max_yaw_acceleration*c.dt,
                                       self.U[1, k]-self.U[1, k-1], c.max_yaw_acceleration*c.dt))
            # Samples within each interval guard against crossing between knots.
            for fraction in (0.25, 0.5, 0.75, 1.0):
                point = integrate(x, u, fraction*c.dt)
                for peer_parameter, distance_parameter in zip(self.peers, self.distances):
                    peer = (1-fraction)*peer_parameter[:, k]+fraction*peer_parameter[:, k+1]
                    o.subject_to(ca.sumsqr(point[:2]-peer) >= distance_parameter[0, k+1]**2)
                if c.bounds:
                    xmin, xmax, ymin, ymax = c.bounds
                    o.subject_to(o.bounded(xmin+c.planning_wall_inset, point[0], xmax-c.planning_wall_inset))
                    o.subject_to(o.bounded(ymin+c.planning_wall_inset, point[1], ymax-c.planning_wall_inset))
                for ox, oy, radius in c.obstacles:
                    o.subject_to(ca.sumsqr(point[:2]-ca.DM([ox, oy])) >= (radius+c.static_padding)**2)
            remaining = ca.sqrt(ca.sumsqr(x[:2]-self.goal[:2])+1e-8)
            desired_speed = ca.fmin(c.max_speed, 0.7*remaining)
            if c.allow_reverse:
                desired_speed *= ((self.goal[0]-x[0])*ca.cos(x[2])+(self.goal[1]-x[1])*ca.sin(x[2]))/remaining
            objective += 5*ca.sumsqr(x[:2]-self.reference[:, k]) + 2*(x[3]-desired_speed)**2
            objective += 0.34*u[0]**2 + 0.85*u[1]**2
            last_rate = self.previous_rate if k == 0 else self.U[1, k-1]
            objective += 2*(u[1]-last_rate)**2
        terminal = self.X[:, -1]
        objective += 15*ca.sumsqr(terminal[:2]-self.goal[:2]) + 5*terminal[3]**2
        objective += 0.5*(1-ca.cos(terminal[2]-self.goal[2]))
        o.minimize(objective)
        options={"print_level":0,"sb":"yes","max_iter":150,"mu_strategy":c.mu_strategy,
                 "tol":1e-4,"acceptable_tol":1e-3,"acceptable_iter":3}
        if c.solve_seconds > 0:
            options.update(max_cpu_time=c.solve_seconds,max_wall_time=c.solve_seconds)
        o.solver("ipopt", {"expand":True,"print_time":False}, options)


    def reset(self):
        self.previous = None

    def solve(self, state, goal, peer: ObjectState, stamp, yaw_rate=0.0):
        c = self.config
        self.last_solver_ms, self.last_solver_calls = 0., 0
        state, goal = np.asarray(state, dtype=float).copy(), np.asarray(goal, dtype=float)
        if state.shape != (4,) or goal.shape != (3,) or not np.isfinite(state).all() or not np.isfinite(goal).all():
            raise ValueError("Expected finite ego [x,y,yaw,v] and goal [x,y,yaw]")
        objects = (peer,) if isinstance(peer, ObjectState) else tuple(peer)
        if len(objects) != c.peer_count or len({obj.robot_id for obj in objects}) != c.peer_count:
            raise ValueError("Expected one distinct object per configured peer")
        if any(obj.frame != "world" for obj in objects) or not math.isfinite(yaw_rate) or abs(yaw_rate) > c.max_yaw_rate+1e-3:
            raise ValueError("Invalid peer coordinate frame or ego yaw rate")
        # Swerve steering at rest produces millimetres/second of signed odom
        # motion. Treat this small reverse drift as zero in the forward model;
        # larger reverse motion still invalidates the plan.
        minimum = -c.max_speed if c.allow_reverse else 0.
        if state[3] < minimum-(0.05 if c.allow_reverse else 0.01) or state[3] > c.max_speed+0.05:
            raise ValueError("Ego speed is outside the configured model limits")
        state[3] = np.clip(state[3], minimum, c.max_speed)
        predictions = tuple(predict_object(obj, stamp, c.dt, c.horizon, self.prediction_config) for obj in objects)
        position_arrays = [np.asarray(pred.positions)[:, :2].T for pred in predictions]
        all_distances = []
        for pred, positions in zip(predictions, position_arrays):
            clearance = np.linalg.norm(state[:2]-positions[:, 0])
            if c.current_clearance_check_enabled and clearance < c.separation+pred.radii[0]:
                raise ValueError("Insufficient current object clearance")
            margin = min(c.tracking_margin, .5*max(0., clearance-c.separation-pred.radii[0]))
            all_distances.append(c.separation+margin+np.asarray(pred.radii))
        # The seed reacts to any blocking object; every object has hard constraints.
        reference = self.make_reference(state, goal, position_arrays)
        prediction, distances = predictions[0], all_distances[0]
        o = self.opti
        o.set_value(self.initial, state)
        o.set_value(self.goal, goal)
        o.set_value(self.reference, reference)
        for pp, dp, positions, limits in zip(self.peers, self.distances, position_arrays, all_distances):
            o.set_value(pp, positions)
            o.set_value(dp, limits.reshape(1, -1))
        o.set_value(self.previous_rate, yaw_rate)
        # Shift only this robot's solution. No peer control/goal/solution exists here.
        if self.previous is not None and 0 < stamp-self.previous.stamp <= 2*c.dt:
            old = self.previous
            # Resample at the actual elapsed simulation time (control dt != horizon dt).
            age = stamp-old.stamp
            seed = np.empty_like(old.states)
            inputs = np.empty_like(old.controls)
            for k in range(c.horizon+1):
                t = min(age+k*c.dt, c.horizon*c.dt)
                index = min(int(t/c.dt), c.horizon-1)
                seed[:, k] = np.asarray(integrate(old.states[:, index], old.controls[:, index], t-index*c.dt)).ravel()
                if k < c.horizon:
                    inputs[:, k] = old.controls[:, index]
            seed[:, 0] = state
        else:
            seed = np.tile(state[:, None], (1, c.horizon+1))
            seed[:2] = reference
            delta = np.diff(reference, axis=1)
            # Gazebo yaw wraps at +/-pi. Anchor the reference headings to the
            # measured branch; a -pi start followed by +pi seeds creates a
            # spurious full turn that stalls IPOPT after each cold reset.
            seed[2, 1:] = np.unwrap(np.r_[state[2], np.arctan2(delta[1], delta[0])])[1:]
            seed[2, 1:] += 2*np.pi*round((state[2]-seed[2, 1])/(2*np.pi))
            seed[3, 1:] = np.minimum(c.max_speed, np.linalg.norm(delta, axis=0)/c.dt)
            if c.allow_reverse and np.dot(delta[:, 0], [math.cos(state[2]), math.sin(state[2])]) < 0:
                seed[2, 1:] = np.unwrap(np.r_[state[2], np.arctan2(delta[1], delta[0])+np.pi])[1:]
                seed[3, 1:] *= -1
            inputs = np.zeros((2, c.horizon))
            inputs[0] = np.clip(np.diff(seed[3])/c.dt, -c.max_acceleration, c.max_acceleration)
            inputs[1] = np.clip(np.diff(seed[2])/c.dt, -c.max_yaw_rate, c.max_yaw_rate)
            if c.bounds and min(c.bounds[1]-c.bounds[0], c.bounds[3]-c.bounds[2])-2*c.planning_wall_inset < c.separation:
                # A side-passing reference crosses the peer in this corridor.
                # That cold guess can trap IPOPT in local infeasibility even
                # when a stationary trajectory is feasible. Start from a
                # dynamically consistent braking rollout; retain the same
                # objective, constraints and solver budget.
                seed[:, 0] = state
                last_rate = yaw_rate
                for k in range(c.horizon):
                    acceleration = np.clip(-seed[3, k]/c.dt, -c.max_acceleration, c.max_acceleration)
                    step = c.max_yaw_acceleration*(c.command_dt if k == 0 else c.dt)
                    rate = last_rate+np.clip(-last_rate, -step, step)
                    inputs[:, k] = acceleration, rate
                    seed[:, k+1] = np.asarray(integrate(seed[:, k], inputs[:, k], c.dt)).ravel()
                    last_rate = rate
        o.set_initial(self.X, seed)
        o.set_initial(self.U, inputs)
        started = time.monotonic()
        try:
            self.last_solver_calls = 1
            try:
                solution = o.solve()
            finally:
                self.last_solver_ms = (time.monotonic()-started)*1000.
            result = Plan(stamp, solution.value(self.X), solution.value(self.U), prediction,
                          time.monotonic()-started, float(solution.value(o.f)), goal.copy(), predictions, tuple(all_distances))
            if not np.isfinite(result.states).all() or not np.isfinite(result.controls).all():
                raise ValueError("Nonfinite solver output")
            self.validate(result, distances)
        except Exception:
            self.reset()
            raise
        self.previous = result
        return result

    def make_reference(self, state, goal, peer_positions):
        """Goal progress with a right-passing initial guess around a blocking peer."""
        c = self.config
        diff = goal[:2]-state[:2]
        length = np.linalg.norm(diff)
        direction = diff/max(length, 1e-6)
        progress = np.minimum(np.arange(c.horizon+1)*c.dt*c.max_speed, length)
        line = state[:2, None]+direction[:, None]*progress
        # Break the symmetric head-on stationary point without using peer intent.
        arrays = [peer_positions] if isinstance(peer_positions, np.ndarray) else peer_positions
        close = any(np.min(np.linalg.norm(line-positions, axis=0)) < c.separation+0.6 for positions in arrays)
        if close and length > 1:
            right = np.array([direction[1], -direction[0]])
            offset = 1.4*np.sin(np.pi*np.minimum(progress/length, 1))
            line += right[:, None]*offset
        if c.bounds:
            xmin, xmax, ymin, ymax = c.bounds
            line[0] = np.clip(line[0], xmin+c.planning_wall_inset, xmax-c.planning_wall_inset)
            line[1] = np.clip(line[1], ymin+c.planning_wall_inset, ymax-c.planning_wall_inset)
        return line

    def validate(self, plan, distances=None):
        """Independent dense motion check before giving a plan to the executor."""
        c = self.config
        if np.max(np.abs(plan.controls[0])) > c.max_acceleration+1e-3 or np.max(np.abs(plan.controls[1])) > c.max_yaw_rate+1e-3:
            raise ValueError("Solver output exceeds input limits")
        if np.min(plan.states[3]) < (-c.max_speed if c.allow_reverse else 0)-1e-3 or np.max(plan.states[3]) > c.max_speed+1e-3:
            raise ValueError("Solver output exceeds speed limits")
        if c.bounds:
            for k in range(c.horizon+1):
                control = plan.controls[:, k] if k < c.horizon else (0., 0.)
                duration = c.dt if k < c.horizon else 0.
                if not workspace_interval_safe(plan.states[:, k], control, duration, c.bounds, c.wall_inset):
                    raise ValueError("Predicted workspace interval rejected by executor")
        predictions = plan.predictions or (plan.prediction,)
        limits = plan.peer_distances or (np.asarray(distances if distances is not None else c.separation+np.asarray(plan.prediction.radii)),)
        peer_arrays = [np.asarray(pred.positions)[:, :2] for pred in predictions]
        for k in range(c.horizon):
            next_state = plan.states[:, k+1]
            remaining = float(np.linalg.norm(next_state[:2]-plan.goal[:2]))
            allowed = max(goal_speed_limit(remaining, c), (abs(plan.states[3, 0]) if c.allow_reverse else plan.states[3, 0])-c.max_acceleration*(k+1)*c.dt)
            if (abs(next_state[3]) if c.allow_reverse else next_state[3]) > allowed+1e-3:
                raise ValueError('Solver output exceeds goal braking envelope')
            target = np.asarray(integrate(plan.states[:, k], plan.controls[:, k], c.dt)).ravel()
            if np.max(np.abs(target-plan.states[:, k+1])) > 1e-3:
                raise ValueError("Solver output violates dynamics")
            for fraction in np.linspace(0, 1, 21):
                point = np.asarray(integrate(plan.states[:, k], plan.controls[:, k], fraction*c.dt)).ravel()
                for peer, distances in zip(peer_arrays, limits):
                    other = (1-fraction)*peer[k]+fraction*peer[k+1]
                    distance = (1-fraction)*distances[k]+fraction*distances[k+1]
                    if np.linalg.norm(point[:2]-other) < distance-1e-3:
                        raise ValueError("Predicted object collision between MPC samples")
                for ox, oy, radius in c.obstacles:
                    if np.linalg.norm(point[:2]-[ox, oy]) < radius+c.static_padding-1e-3:
                        raise ValueError("Predicted static obstacle collision")
