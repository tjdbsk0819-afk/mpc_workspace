"""Hard rectangular workspace constraints at initial, path and terminal states."""
import numpy as np
from control_modules import ConstraintModule


class WorkspaceBoundsModule(ConstraintModule):
    def __init__(self, settings):
        super().__init__()
        self.module_name = 'WorkspaceBounds'
        self.import_name = 'workspace_bounds.h'
        self.description = 'Wall-safe circular robot footprint inside a rectangular workspace'
        self.constraints.append(WorkspaceBoundsConstraint())


class WorkspaceBoundsConstraint:
    nh = 4
    apply_at_initial_stage = True
    apply_at_terminal_stage = True

    def define_parameters(self, params):
        for key in ('workspace_xmin', 'workspace_xmax', 'workspace_ymin', 'workspace_ymax'):
            params.add(key)

    def get_lower_bound(self):
        return [0.] * 4

    def get_upper_bound(self):
        return [np.inf] * 4

    def get_constraints(self, model, params, settings, stage_idx):
        return [model.get('x')-params.get('workspace_xmin'),
                params.get('workspace_xmax')-model.get('x'),
                model.get('y')-params.get('workspace_ymin'),
                params.get('workspace_ymax')-model.get('y')]
