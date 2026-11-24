"""Goal approach and stationary final heading inside the original unicycle OCP."""
import casadi as cd
import numpy as np
from control_modules import Objective, ConstraintModule


class GoalArrivalObjective(Objective):
    def define_parameters(self, params):
        # The associated constraint module owns the shared runtime parameters.
        pass

    def get_value(self, model, params, settings, stage_idx):
        dx = model.get('x') - params.get('arrival_x')
        dy = model.get('y') - params.get('arrival_y')
        distance = cd.sqrt(dx*dx + dy*dy + 1e-8)
        cfg = settings['goal_handling']
        reference = cd.fmin(settings['weights']['reference_velocity'],
                            cfg['approach_gain'] * distance)
        reference = cd.fmin(reference, cd.sqrt(2.*cfg['approach_deceleration'] * distance))
        reference *= 1. - params.get('arrival_align')
        terminal = stage_idx == settings['N'] - 1
        position_weight = cfg['terminal_position_weight'] if terminal else cfg['position_weight']
        cost = position_weight * (dx*dx + dy*dy)
        cost += settings['weights']['velocity'] * (model.get('v') - reference)**2
        # Periodic yaw error avoids the +/-pi branch discontinuity.
        cost += params.get('arrival_align') * cfg['heading_weight'] * (
            1. - cd.cos(model.get('psi') - params.get('arrival_yaw')))
        # A nearby goal behind the robot must not become a stationary local
        # minimum. This direction belongs to the ego goal, not a peer priority.
        facing_away = 1. + (cd.cos(model.get('psi'))*dx + cd.sin(model.get('psi'))*dy)/distance
        cost += (1.-params.get('arrival_align')) * cfg['approach_heading_weight'] * cd.exp(-distance*distance) * facing_away
        return cost


class GoalHandlingModule(ConstraintModule):
    def __init__(self, settings):
        super().__init__()
        self.module_name = 'GoalHandling'
        self.import_name = 'goal_handling.h'
        self.description = 'Distance-based approach cost, final yaw and stationary alignment'
        self.constraints.append(StationaryAlignmentConstraint())


class StationaryAlignmentConstraint:
    nh = 2

    def define_parameters(self, params):
        for key in ('arrival_x', 'arrival_y', 'arrival_yaw', 'arrival_align', 'arrival_speed_cap'):
            params.add(key)

    def get_lower_bound(self):
        return [0., 0.]

    def get_upper_bound(self):
        return [np.inf, np.inf]

    def get_constraints(self, model, params, settings, stage_idx):
        cfg = settings['goal_handling']
        dx = model.get('x') - params.get('arrival_x')
        dy = model.get('y') - params.get('arrival_y')
        distance = cd.sqrt(dx*dx + dy*dy + 1e-8)
        reserve = cd.fmax(distance-cfg['stop_reserve'], 0.)
        speed = model.get('v')
        stopping = speed*speed + 2.*cfg['approach_deceleration']*cfg['reaction_time']*speed
        # Final alignment already constrains translation to zero. Leave the
        # braking inequality inactive there, including tiny solver residuals.
        brake = 2.*cfg['approach_deceleration']*reserve-stopping + params.get('arrival_align')
        return [params.get('arrival_speed_cap') - speed, brake]
