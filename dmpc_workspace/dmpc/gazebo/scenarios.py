"""A/B/C/D: preserve the original two starts/goals and add named robots."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Layout:
    scenario: str
    robots: dict
    bounds: tuple | None = None
    obstacles: tuple = ()

def layout(scenario='B', count=3):
    if scenario not in ('A', 'B', 'C', 'D') or count not in (3, 4):
        raise ValueError('Use scenario A/B/C/D and 3 or 4 robots')
    pi = math.pi
    starts = [(-3.,0.,0.),(3.,0.,pi),(-6.,0.,0.),(6.,0.,pi)]
    goals = [(3.,0.,0.),(-3.,0.,pi),(6.,0.,0.),(-6.,0.,pi)]
    if scenario == 'B':
        starts = [(-3.,0.,0.),(0.,-3.,pi/2),(3.,0.,pi),(0.,3.,-pi/2)]
        goals = [(3.,0.,0.),(0.,3.,pi/2),(-3.,0.,pi),(0.,-3.,-pi/2)]
    if scenario == 'C':
        starts[2:] = [(-1.5,0.,0.),(1.5,0.,pi)]
        goals[2:] = [(1.5,0.,0.),(-1.5,0.,pi)]
    robots = {f'robot_{chr(97+i)}': {'start':starts[i], 'goal':goals[i]} for i in range(count)}
    return Layout(scenario, robots, (-4.,4.,-.7,.7) if scenario=='C' else None,
                  ((0.,0.,.4),) if scenario=='D' else ())

def workspace(scene):
    return dict(workspace_bounds_enabled=scene.bounds is not None,
                workspace_bounds=list(scene.bounds or (-2000.,2000.,-2000.,2000.)),
                wall_robot_radius=.5, wall_safety_margin=.05,
                static_obstacle_enabled=bool(scene.obstacles),
                static_obstacle=list(scene.obstacles[0] if scene.obstacles else (0.,0.,.4)),
                static_robot_radius=.5, static_safety_margin=.15)
