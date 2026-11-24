#!/usr/bin/env python3
"""Upstream AntBot OCP plus previous-plan yaw cost and first-input envelope."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'solver_generator'), str(ROOT / 'mpc_planner_modules/scripts')]

from util.files import load_settings
from control_modules import ModuleManager
from generate_solver import generate_solver
from solver_model import ContouringSecondOrderUnicycleModel
from mpc_base import MPCBaseModule
from contouring import ContouringModule
from ellipsoid_constraints import EllipsoidConstraintModule
from yaw_continuity import YawContinuityObjective, YawRateEnvelopeModule
from goal_handling import GoalArrivalObjective, GoalHandlingModule
from workspace_bounds import WorkspaceBoundsModule
from static_obstacle import StaticObstacleModule


def main():
    settings = load_settings()
    model = ContouringSecondOrderUnicycleModel()
    limits = settings['antbot_limits']
    # Upstream order: [a, w, x, y, psi, v, spline].
    model.set_bounds(
        [-limits['acceleration'], -limits['yaw_rate'], -2000., -2000., -12.5664, 0., -1.],
        [limits['acceleration'], limits['yaw_rate'], 2000., 2000., 12.5664, limits['speed'], 10000.],
    )
    modules = ModuleManager()
    base = modules.add_module(MPCBaseModule(settings))
    base.weigh_variable('a', 'acceleration')
    base.weigh_variable('w', 'angular_velocity')
    base.objectives.append(GoalArrivalObjective())
    base.objectives.append(YawContinuityObjective())
    modules.add_module(ContouringModule(settings))
    modules.add_module(EllipsoidConstraintModule(settings))
    modules.add_module(YawRateEnvelopeModule(settings))
    modules.add_module(GoalHandlingModule(settings))
    modules.add_module(WorkspaceBoundsModule(settings))
    modules.add_module(StaticObstacleModule(settings))
    generate_solver(modules, model, settings)


if __name__ == '__main__':
    main()
