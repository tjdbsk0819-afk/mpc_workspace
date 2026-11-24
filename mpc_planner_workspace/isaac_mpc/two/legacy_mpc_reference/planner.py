"""Centralized joint NLP and independent per-robot NLPs using CasADi/IPOPT.

Adapts upstream src/distributed.py and src/decentralized.py: build once,
update parameters, solve, apply first command, shift primal initial guesses.
No importing upstream utils is necessary (its simulation layer imports MuJoCo).
"""
from dataclasses import dataclass
from time import perf_counter
import casadi as ca
import numpy as np
from .model import step_symbolic, step, segment_distance


@dataclass
class Plan:
    states: np.ndarray  # (robots, N+1, 3)
    inputs: np.ndarray  # (robots, N, 3)
    success: bool
    seconds: float
    status: str

    @property
    def command(self):
        return self.inputs[:, 0]


class Planner:
    def __init__(self, config, robots=2, obstacles=(), bounds=None):
        if robots not in (1, 2):
            raise ValueError('This experiment supports one local robot or two joint robots')
        self.cfg, self.robots = config, robots
        self.obstacles = tuple(obstacles)
        self.bounds = bounds
        self._warm = None
        self.opti = o = ca.Opti()
        n = config.horizon
        self.X = o.variable(3*robots, n+1)
        self.U = o.variable(3*robots, n)
        self.x0 = o.parameter(3*robots)
        self.target = o.parameter(3*robots)
        self.previous = o.parameter(3*robots)
        self.peer = o.parameter(3, n+1) if robots == 1 else None
        self.inflation = o.parameter(1, n+1) if robots == 1 else None
        o.subject_to(self.X[:, 0] == self.x0)
        cost = 0
        for m in range(robots):
            xs = self.X[3*m:3*m+3, :]
            us = self.U[3*m:3*m+3, :]
            goal = self.target[3*m:3*m+3]
            for j in range(3):
                o.subject_to(o.bounded(-config.velocity[j], us[j, :], config.velocity[j]))
            for k in range(n):
                x, u = xs[:, k], us[:, k]
                prev = self.previous[3*m:3*m+3] if k == 0 else us[:, k-1]
                du = u-prev
                for j in range(3):
                    limit = config.acceleration[j]*config.dt
                    o.subject_to(o.bounded(-limit, du[j], limit))
                error = ca.vertcat(x[:2]-goal[:2], ca.atan2(ca.sin(x[2]-goal[2]), ca.cos(x[2]-goal[2])))
                cost += ca.dot(ca.DM(config.q), error**2)
                cost += ca.dot(ca.DM(config.r), u**2)+ca.dot(ca.DM(config.delta_r), du**2)
                o.subject_to(xs[:, k+1] == step_symbolic(x, u, config.dt))
            error = ca.vertcat(xs[:2, -1]-goal[:2], ca.atan2(ca.sin(xs[2, -1]-goal[2]), ca.cos(xs[2, -1]-goal[2])))
            cost += ca.dot(ca.DM(config.terminal_q), error**2)
            # Additional samples between knots limit discretization blind spots.
            for k in range(n):
                for f in (0., 0.5, 1.):
                    p = step_symbolic(xs[:, k], us[:, k], config.dt*f)[:2]
                    for ox, oy, radius in self.obstacles:
                        o.subject_to(ca.sumsqr(p-ca.DM([ox, oy])) >= (config.radius+config.margin+radius)**2)
                    if bounds is not None:
                        xmin, xmax, ymin, ymax = bounds
                        o.subject_to(o.bounded(xmin+config.radius, p[0], xmax-config.radius))
                        o.subject_to(o.bounded(ymin+config.radius, p[1], ymax-config.radius))
        for k in range(n):
            for f in (0., 0.5, 1.):
                a = step_symbolic(self.X[:3, k], self.U[:3, k], config.dt*f)[:2]
                if robots == 2:
                    b = step_symbolic(self.X[3:6, k], self.U[3:6, k], config.dt*f)[:2]
                    distance = config.separation
                else:
                    b = (1-f)*self.peer[:2, k]+f*self.peer[:2, k+1]
                    distance = config.separation+(1-f)*self.inflation[0, k]+f*self.inflation[0, k+1]
                o.subject_to(ca.sumsqr(a-b) >= distance**2)
        o.minimize(cost)
        # Flatten the expression graph once; repeatedly evaluating nested MX
        # RK4 expressions was consuming the entire ROS solve budget.
        o.solver('ipopt', {'print_time': False, 'expand': True}, {
            'print_level': 0, 'sb': 'yes', 'max_iter': config.solver_max_iterations,
            'max_wall_time': config.solver_max_seconds, 'tol': 1e-6,
        })

    def stop(self, states, status, seconds=0.):
        states = np.asarray(states).reshape(self.robots, 3)
        self._warm = None
        return Plan(np.repeat(states[:, None, :], self.cfg.horizon+1, axis=1),
                    np.zeros((self.robots, self.cfg.horizon, 3)), False, seconds, status)

    def solve(self, states, targets, previous, peer=None, inflation=None):
        states = np.asarray(states, float).reshape(self.robots, 3)
        targets = np.asarray(targets, float).reshape(self.robots, 3)
        previous = np.asarray(previous, float).reshape(self.robots, 3)
        if not all(np.isfinite(z).all() for z in (states, targets, previous)):
            return self.stop(states, 'invalid_state')
        o, cfg = self.opti, self.cfg
        if self.robots == 1:
            if peer is None:
                return self.stop(states, 'missing_peer_prediction')
            peer = np.asarray(peer, float)
            inflation = np.zeros(cfg.horizon+1) if inflation is None else np.asarray(inflation, float)
            if (peer.shape != (cfg.horizon+1, 3) or inflation.shape != (cfg.horizon+1,)
                    or not np.isfinite(peer).all() or not np.isfinite(inflation).all() or (inflation < 0).any()):
                return self.stop(states, 'invalid_peer_prediction')
            o.set_value(self.peer, peer.T)
            o.set_value(self.inflation, inflation.reshape(1, -1))
        o.set_value(self.x0, states.reshape(-1))
        o.set_value(self.target, targets.reshape(-1))
        o.set_value(self.previous, previous.reshape(-1))
        if self._warm is not None:
            px, pu = self._warm
            o.set_initial(self.X, np.hstack([px[:, 1:], px[:, -1:]]))
            o.set_initial(self.U, np.hstack([pu[:, 1:], pu[:, -1:]]))
        else:
            # Feasible dynamic rollout toward the route waypoint; stronger seed
            # than linear state interpolation at a symmetric encounter.
            px = np.zeros((3*self.robots, cfg.horizon+1))
            pu = np.zeros((3*self.robots, cfg.horizon))
            for m in range(self.robots):
                x, u = states[m].copy(), previous[m].copy()
                px[3*m:3*m+3, 0] = x
                for k in range(cfg.horizon):
                    d = targets[m, :2]-x[:2]
                    c, s = np.cos(x[2]), np.sin(x[2])
                    desired = np.array([c*d[0]+s*d[1], -s*d[0]+c*d[1], 0.])
                    desired[:2] *= min(1., cfg.velocity[0]/max(np.linalg.norm(d), 1e-9))
                    desired[2] = np.arctan2(np.sin(targets[m, 2]-x[2]), np.cos(targets[m, 2]-x[2]))
                    u += np.clip(desired-u, -np.array(cfg.acceleration)*cfg.dt, np.array(cfg.acceleration)*cfg.dt)
                    u = np.clip(u, -np.array(cfg.velocity), np.array(cfg.velocity))
                    x = step(x, u, cfg.dt)
                    pu[3*m:3*m+3, k] = u
                    px[3*m:3*m+3, k+1] = x
            o.set_initial(self.X, px)
            o.set_initial(self.U, pu)
        start = perf_counter()
        try:
            sol = o.solve()
            px, pu = np.asarray(sol.value(self.X)), np.asarray(sol.value(self.U))
            elapsed = perf_counter()-start
            lower, upper = np.asarray(sol.value(o.lbg)).reshape(-1), np.asarray(sol.value(o.ubg)).reshape(-1)
            g = np.asarray(sol.value(o.g)).reshape(-1)
            residual = max(np.max(lower-g), np.max(g-upper))
            if not np.isfinite(px).all() or not np.isfinite(pu).all() or residual > 1e-4:
                return self.stop(states, 'constraint_check_failed', elapsed)
            self._warm = px.copy(), pu.copy()
            return Plan(px.reshape(self.robots, 3, -1).transpose(0, 2, 1),
                        pu.reshape(self.robots, 3, -1).transpose(0, 2, 1),
                        True, elapsed, sol.stats()['return_status'])
        except RuntimeError as error:
            return self.stop(states, 'solver_failed: '+str(error).splitlines()[-1], perf_counter()-start)


def command_is_safe(states, commands, cfg, obstacles=(), bounds=None):
    """Short-step veto, not a proof of safety under actuator lag or unbounded peers."""
    samples = [np.asarray(states, float)]
    for k in range(1, 11):
        samples.append(np.array([step(x, u, cfg.dt*k/10) for x, u in zip(states, commands)]))
    for before, after in zip(samples[:-1], samples[1:]):
        if segment_distance(before[0], after[0], before[1], after[1]) < cfg.separation-1e-5:
            return False
        for m in range(2):
            for ox, oy, radius in obstacles:
                p = np.array([ox, oy, 0.])
                if segment_distance(before[m], after[m], p, p) < cfg.radius+cfg.margin+radius-1e-5:
                    return False
            if bounds:
                xmin, xmax, ymin, ymax = bounds
                if not (xmin+cfg.radius <= after[m, 0] <= xmax-cfg.radius
                        and ymin+cfg.radius <= after[m, 1] <= ymax-cfg.radius):
                    return False
    return True
