"""Shared spawn poses and goal poses for the two-robot Isaac/MPC launches."""
from dataclasses import dataclass
import math

DEFAULT_SCENARIO = 'B'
GOAL_ARGUMENTS = tuple(f'{robot}_goal_{axis}'
                       for robot in ('robot_a', 'robot_b') for axis in ('x', 'y', 'yaw'))
DRIVING_SCENARIOS = ['A', 'B', 'C', 'D']
WALL_ROBOT_RADIUS = .5
WALL_SAFETY_MARGIN = .05
STATIC_SAFETY_MARGIN = .15
STATIC_TRACKING_MARGIN = .05


@dataclass(frozen=True)
class ScenarioLayout:
    name: str
    starts: tuple
    goals: tuple
    obstacles: tuple = ()
    bounds: tuple | None = None


def scenario_layout(name=DEFAULT_SCENARIO):
    starts = ((-3., 0., 0.), (3., 0., math.pi))
    goals = ((3., 0., 0.), (-3., 0., math.pi))
    if name == 'A':
        return ScenarioLayout(name, starts, goals)
    if name == 'B':
        return ScenarioLayout(name, ((-3., 0., 0.), (0., -3., math.pi/2)),
                              ((3., 0., 0.), (0., 3., math.pi/2)))
    if name == 'C':
        return ScenarioLayout(name, starts, goals, bounds=(-4., 4., -.7, .7))
    if name == 'D':
        return ScenarioLayout(name, starts, goals, obstacles=((0., 0., .4),))
    raise ValueError(f'Unknown scenario {name}')


def planner_goals(name=DEFAULT_SCENARIO, overrides=None):
    if name not in DRIVING_SCENARIOS:
        raise ValueError('Driving supports scenarios A/B/C/D')
    layout = scenario_layout(name)
    defaults = dict(zip(GOAL_ARGUMENTS, (*layout.goals[0], *layout.goals[1])))
    overrides = overrides or {}
    goals = {key: float(value if overrides.get(key) in (None, '') else overrides[key])
             for key, value in defaults.items()}
    if not all(math.isfinite(value) for value in goals.values()):
        raise ValueError('Goal poses must be finite')
    if layout.bounds:
        xmin, xmax, ymin, ymax = layout.bounds
        inset = WALL_ROBOT_RADIUS + WALL_SAFETY_MARGIN
        for robot in ('robot_a', 'robot_b'):
            if not (xmin+inset <= goals[f'{robot}_goal_x'] <= xmax-inset and
                    ymin+inset <= goals[f'{robot}_goal_y'] <= ymax-inset):
                raise ValueError(f'{robot} goal is outside the wall-safe corridor')
    for x, y, radius in layout.obstacles:
        separation = radius + WALL_ROBOT_RADIUS + STATIC_SAFETY_MARGIN + STATIC_TRACKING_MARGIN
        for robot in ('robot_a', 'robot_b'):
            if math.hypot(goals[f'{robot}_goal_x']-x, goals[f'{robot}_goal_y']-y) < separation:
                raise ValueError(f'{robot} goal is inside a static obstacle safety area')
    return goals


def workspace_parameters(name=DEFAULT_SCENARIO):
    layout = scenario_layout(name)
    bounds = layout.bounds
    if len(layout.obstacles)>1:
        raise ValueError('The two-robot static OCP currently has one fixed obstacle slot')
    return dict(workspace_bounds_enabled=bounds is not None,
                workspace_bounds=list(bounds or (-2000., 2000., -2000., 2000.)),
                wall_robot_radius=WALL_ROBOT_RADIUS, wall_safety_margin=WALL_SAFETY_MARGIN,
                static_obstacle_enabled=bool(layout.obstacles),
                static_obstacle=list(layout.obstacles[0] if layout.obstacles else (0., 0., .4)),
                static_robot_radius=WALL_ROBOT_RADIUS, static_safety_margin=STATIC_SAFETY_MARGIN)
