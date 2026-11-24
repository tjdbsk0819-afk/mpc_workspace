from pathlib import Path
from dataclasses import replace
import shutil
import subprocess
import tempfile
import unittest

import numpy as np
import yaml

from dmpc_runtime.planner import LocalObjectMPC, PlannerConfig, integrate
from dmpc_runtime.prediction import ObjectState


@unittest.skipUnless(shutil.which("g++"), "g++ is needed for the native RPP contract check")
class ExecutorContractTest(unittest.TestCase):
    def test_corridor_plans_pass_the_actual_native_wall_guard(self):
        root = Path(__file__).resolve().parents[1]
        params = yaml.safe_load((root/'isaac/controller_bringup/config/dmpc.yaml').read_text())['/**']['ros__parameters']
        config = PlannerConfig(**{k: v for k, v in params.items() if k in PlannerConfig.__dataclass_fields__},
                               bounds=(-4., 4., -.7, .7))
        with tempfile.TemporaryDirectory(prefix='dmpc-wall-contract-') as temp:
            binary = str(Path(temp)/'check')
            subprocess.run(['g++', '-std=c++17', '-O2', '-I',
                            str(root/'isaac/ros_messages/antbot_interfaces/include'),
                            str(root/'tests/workspace_contract.cpp'), '-o', binary], check=True, capture_output=True)
            for state, goal, position in (([-3., 0., 0., 0.], [3., 0., 0.], (3., 0., 0.)),
                                          ([3., 0., np.pi, 0.], [-3., 0., np.pi], (-3., 0., 0.))):
                peer = ObjectState('peer', 0., position, (0., 0., 0.))
                for margin, accepted in ((0., False), (config.wall_planning_margin, True)):
                    with self.subTest(initial=state, planning_margin=margin):
                        planner = LocalObjectMPC(replace(config, wall_planning_margin=margin))
                        validate = planner.validate
                        if not accepted:
                            # Recover the old raw OCP result to demonstrate its
                            # rejection by native RPP and the new publish guard.
                            planner.validate = lambda *args: None
                        plan = planner.solve(state, goal, peer, 0.)
                        if not accepted:
                            with self.assertRaisesRegex(ValueError, 'workspace interval rejected by executor'):
                                validate(plan)
                        lines = [f'{config.horizon+1} {config.dt} {int(accepted)}']
                        for k in range(config.horizon+1):
                            u = plan.controls[:, k] if k<config.horizon else (0., 0.)
                            lines.append(' '.join(map(str, [*plan.states[:, k], *u])))
                        subprocess.run([binary], input='\n'.join(lines)+'\n', text=True,
                                       capture_output=True, check=True)

    def test_real_ipopt_output_matches_native_rpp_sampling(self):
        root = Path(__file__).resolve().parents[1]
        c = PlannerConfig(horizon=15, prediction_model="cv", solve_seconds=1)
        plan = LocalObjectMPC(c).solve([-2, 0, 0, .1], [2, 1, .3],
                                      ObjectState("b", 0, (0, -3, 0), (0, 0, 0)), 0)
        lines = [f"{c.horizon+1} {c.dt}"]
        for k in range(c.horizon+1):
            u = plan.controls[:, k] if k < c.horizon else [0, 0]
            lines.append(" ".join(map(str, [*plan.states[:, k], *u])))
        for k in range(c.horizon):
            middle = np.asarray(integrate(plan.states[:, k], plan.controls[:, k], c.dt/2)).ravel()
            lines.append(" ".join(map(str, middle)))
        with tempfile.TemporaryDirectory(prefix="dmpc-contract-") as temp:
            binary = str(Path(temp)/"check")
            subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(root/"isaac/ros_messages/antbot_interfaces/include"),
                            str(root/"tests/executor_contract.cpp"), "-o", binary], check=True, capture_output=True)
            subprocess.run([binary], input="\n".join(lines)+"\n", text=True, check=True, capture_output=True)
