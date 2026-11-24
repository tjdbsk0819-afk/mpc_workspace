"""AntBot: continuity against the previous selected plan, inside the OCP."""
import numpy as np
from control_modules import Objective, ConstraintModule


class YawContinuityObjective(Objective):
    def define_parameters(self, params):
        params.add('yaw_reference')
        params.add('yaw_continuity_weight')

    def get_value(self, model, params, settings, stage_idx):
        return params.get('yaw_continuity_weight') * (model.get('w')-params.get('yaw_reference'))**2


class YawRateEnvelopeModule(ConstraintModule):
    def __init__(self, settings):
        super().__init__()
        self.module_name = 'YawRateEnvelope'
        self.import_name = 'yaw_rate_envelope.h'
        self.description = 'Measured first-command yaw envelope and previous-plan yaw cost parameters'
        self.constraints.append(YawRateEnvelopeConstraint())


class YawRateEnvelopeConstraint:
    nh = 2
    apply_at_initial_stage = True

    def define_parameters(self, params):
        params.add('yaw_lower')
        params.add('yaw_upper')

    def get_lower_bound(self):
        return [0., 0.]

    def get_upper_bound(self):
        return [np.inf, np.inf]

    def get_constraints(self, model, params, settings, stage_idx):
        return [model.get('w')-params.get('yaw_lower'),
                params.get('yaw_upper')-model.get('w')]
