"""One known stationary circular obstacle, independent of the probabilistic peer."""
import casadi as cd
import numpy as np
from control_modules import ConstraintModule


class StaticObstacleModule(ConstraintModule):
    def __init__(self, settings):
        super().__init__()
        self.module_name = 'StaticCircleConstraints'
        self.import_name = 'static_obstacle.h'
        self.description = 'Known stationary cylinder plus ego footprint and tracking margin'
        self.constraints.append(StaticObstacleConstraint())


class StaticObstacleConstraint:
    nh = 1
    apply_at_initial_stage = True
    apply_at_terminal_stage = True

    def define_parameters(self, params):
        for key in ('static_enabled', 'static_x', 'static_y', 'static_separation'):
            params.add(key)

    def get_lower_bound(self):
        return [0.]

    def get_upper_bound(self):
        return [np.inf]

    def get_constraints(self, model, params, settings, stage_idx):
        dx, dy = model.get('x')-params.get('static_x'), model.get('y')-params.get('static_y')
        separation = params.get('static_separation')
        return [cd.if_else(params.get('static_enabled')>.5,
                           (dx*dx+dy*dy)/(separation*separation)-1., 1.)]
