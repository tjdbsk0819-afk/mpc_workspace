"""ROS-free two-robot numerical smoke run using state-only observations."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np

from .planner import LocalObjectMPC, PlannerConfig, integrate
from .prediction import ObjectState


def run(scenario="B", duration=30.0, control_dt=0.1, config=None):
    # Installed with dmpc-runtime; importing the scene spec never starts Kit.
    try:
        from simulation.helpers.scenario_config import scenario_layout
    except ModuleNotFoundError:
        from isaac.simulation.helpers.scenario_config import scenario_layout
    layout = scenario_layout(scenario)
    config = config or PlannerConfig(solve_seconds=0.5)
    config = replace(config, command_dt=control_dt, bounds=layout.bounds, obstacles=layout.obstacles)
    planners = [LocalObjectMPC(config) for _ in range(2)]
    states = np.array([(*pose, 0.0) for pose in layout.starts], dtype=float)
    rates = np.zeros(2)
    goals = np.asarray(layout.goals)
    minimum = math.inf
    failures = 0
    solves = []
    history = []
    reached = np.zeros(2, dtype=bool)
    for tick in range(round(duration/control_dt)):
        now = tick*control_dt
        snapshot = states.copy()
        commands = []
        for ego in range(2):
            peer = snapshot[1-ego]
            observed = ObjectState(f"robot_{1-ego}", now, (*peer[:2], 0.0),
                                   (peer[3]*math.cos(peer[2]), peer[3]*math.sin(peer[2]), 0.0))
            reached[ego] = np.linalg.norm(snapshot[ego, :2]-goals[ego, :2]) <= config.goal_tolerance and snapshot[ego, 3] < 0.05
            if reached[ego]:
                command = np.array([0.0, 0.0])
            else:
                try:
                    plan = planners[ego].solve(snapshot[ego], goals[ego], observed, now, rates[ego])
                    command = plan.controls[:, 0]
                    solves.append(plan.elapsed)
                except (RuntimeError, ValueError):
                    failures += 1
                    command = np.array([-min(config.max_acceleration, snapshot[ego, 3]/control_dt),
                                        rates[ego]-np.clip(rates[ego], -config.max_yaw_acceleration*control_dt,
                                                          config.max_yaw_acceleration*control_dt)])
            commands.append(command)
        for fraction in np.linspace(0, 1, 11):
            points = [np.asarray(integrate(snapshot[i], commands[i], control_dt*fraction)).ravel() for i in range(2)]
            minimum = min(minimum, float(np.linalg.norm(points[0][:2]-points[1][:2])))
        for ego in range(2):
            states[ego] = np.asarray(integrate(snapshot[ego], commands[ego], control_dt)).ravel()
            states[ego, 3] = max(0, states[ego, 3])
            rates[ego] = commands[ego][1]
        history.append(dict(t=now+control_dt, states=states.tolist()))
        if reached.all():
            break
    return dict(scenario=scenario, prediction_model=config.prediction_model, minimum_separation=minimum,
                required_separation=config.separation, reached=reached.tolist(), solver_failures=failures,
                solve_median=float(np.median(solves)) if solves else None,
                final_goal_distances=np.linalg.norm(states[:, :2]-goals[:, :2], axis=1).tolist(),
                elapsed_sim_time=len(history)*control_dt, history=history)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=list("ABCD"), default="B")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--prediction", choices=["cv", "cv_gaussian"], default="cv_gaussian")
    parser.add_argument("--solve-seconds", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("Duration must be positive")
    result = run(args.scenario, args.duration, config=PlannerConfig(
        prediction_model=args.prediction, solve_seconds=args.solve_seconds))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({k: v for k, v in result.items() if k != "history"}, indent=2))
    if result["minimum_separation"] < result["required_separation"]-1e-3:
        raise SystemExit("Numerical run violated minimum separation")


if __name__ == "__main__":
    main()
