"""Shared route policy: deterministic right-hand passing breaks symmetry.

Waypoints are a simple upper planning layer, applied equally to both MPC modes.
They do not encode timed trajectories or replace collision constraints.
"""
from dataclasses import dataclass
import numpy as np
from .model import angle_error


@dataclass
class Scenario:
    name: str
    starts: np.ndarray
    goals: np.ndarray
    routes: list
    obstacles: tuple = ()
    bounds: tuple | None = None
    expected_reach: bool = True


def scenario(name='A'):
    starts = np.array([[-3., 0., 0.], [3., 0., np.pi]])
    goals = np.array([[3., 0., 0.], [-3., 0., np.pi]])
    routes = [np.array([[-1., -.8, 0.], [1., -.8, 0.], goals[0]]),
              np.array([[1., .8, np.pi], [-1., .8, np.pi], goals[1]])]
    if name == 'A':
        return Scenario(name, starts, goals, routes)
    if name == 'B':
        starts = np.array([[-3., 0., 0.], [0., -3., np.pi/2]])
        goals = np.array([[3., 0., 0.], [0., 3., np.pi/2]])
        routes = [np.array([[-1.4, -.8, 0.], [1.4, -.8, 0.], goals[0]]),
                  np.array([[-1.4, -1.6, np.pi/2], [-1.4, 1.6, np.pi/2], goals[1]])]
        return Scenario(name, starts, goals, routes)
    if name == 'C':
        # Width 1.4 m admits one robot but not two diameter-1.1 m robots.
        return Scenario(name, starts, goals, [goals[0:1], goals[1:2]],
                        bounds=(-4., 4., -.7, .7), expected_reach=False)
    if name == 'D':
        routes = [np.array([[-1.5, -1.4, 0.], [1.5, -1.4, 0.], goals[0]]),
                  np.array([[1.5, 1.4, np.pi], [-1.5, 1.4, np.pi], goals[1]])]
        return Scenario(name, starts, goals, routes, obstacles=((0., 0., .4),))
    raise ValueError(f'Unknown scenario {name}')


class RouteTracker:
    def __init__(self, spec, cfg):
        self.spec, self.cfg = spec, cfg
        self.indices = [0, 0]

    def target(self, robot, state):
        route = self.spec.routes[robot]
        index = self.indices[robot]
        if index < len(route)-1 and np.linalg.norm(state[:2]-route[index, :2]) < self.cfg.waypoint_tolerance:
            self.indices[robot] += 1
        return route[self.indices[robot]]

    def reached(self, robot, state):
        goal = self.spec.goals[robot]
        return (np.linalg.norm(state[:2]-goal[:2]) < self.cfg.goal_tolerance
                and abs(angle_error(state[2], goal[2])) < self.cfg.yaw_tolerance)
