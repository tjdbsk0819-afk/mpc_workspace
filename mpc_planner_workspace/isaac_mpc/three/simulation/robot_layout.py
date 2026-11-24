"""Shared three-robot spawn/goal configuration for Isaac and ROS launch."""
from copy import deepcopy
from pathlib import Path
import math
import yaml

DEFAULT_LAYOUT = Path(__file__).with_name('three_robots.yaml')

def load_layout(path=DEFAULT_LAYOUT, scenario='staggered'):
    document = yaml.safe_load(Path(path).read_text())
    robots = deepcopy(document['robots'])
    if tuple(robots) != ('robot_a', 'robot_b', 'robot_c'):
        raise ValueError('Layout must define robot_a, robot_b, robot_c in order')
    if scenario not in document['scenarios']:
        raise ValueError(f'Unknown scenario: {scenario}')
    for key,value in document['scenarios'][scenario].items():
        robot,field=key.rsplit('_',1);robots[robot][field]=value
    for name,config in robots.items():
        for field,count in [('start',3),('goal',2)]:
            if len(config[field]) != count or not all(math.isfinite(x) for x in config[field]):
                raise ValueError(f'Invalid {name} {field}')
    for i,a in enumerate(robots):
        for b in list(robots)[i+1:]:
            if math.dist(robots[a]['start'][:2],robots[b]['start'][:2]) < 1.15:
                raise ValueError(f'Start poses too close: {a}, {b}')
    return robots
