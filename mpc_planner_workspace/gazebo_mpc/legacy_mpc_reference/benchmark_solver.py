"""Reproduce the ROS waypoint-transition state and check the IPOPT time budget."""
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np
from .config import MPCConfig
from .planner import Planner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--config')
    parser.add_argument('--output', default='antbot_results/solver_budget_check.json')
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    cfg = MPCConfig.load(args.config)
    planner = Planner(cfg, 2)
    # As in SolverWorker, plugin initialization occurs before timed requests.
    initial = np.array([[-2., 0., 0.], [2., 0., 0.]])
    planner.solve(initial, initial, np.zeros((2, 3)))
    states = np.array([[-1.0381653, -.8521995, .0039127],
                       [1.0381653, .8521995, 3.14550535]])
    targets = np.array([[1., -.8, 0.], [-1., .8, np.pi]])
    samples = []
    for attempt in range(args.repeats):
        # Include cold initial guesses, since the failed ROS loop cleared them.
        planner._warm = None
        result = planner.solve(states, targets, np.zeros((2, 3)))
        sample = {'attempt': attempt, 'success': result.success,
                  'seconds': result.seconds, 'status': result.status,
                  'within_control_period': result.seconds < cfg.dt}
        samples.append(sample)
        print(json.dumps(sample), flush=True)
    report = {'control_period_seconds': cfg.dt, 'ipopt_budget_seconds': cfg.solver_max_seconds,
              'horizon': cfg.horizon, 'samples': samples}
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2)+'\n')
    if any(not s['success'] or not s['within_control_period'] for s in samples):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
