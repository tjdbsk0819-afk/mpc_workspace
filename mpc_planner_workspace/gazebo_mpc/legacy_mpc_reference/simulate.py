"""Deterministic two-robot benchmark, timestamped delayed prediction transport."""
import argparse
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path

# Keep solver threads bounded before importing numerical libraries.
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np
from .config import MPCConfig
from .model import step, segment_distance
from .planner import Planner, command_is_safe
from .scenarios import scenario, RouteTracker
from .trajectory import Prediction, hold_prediction


def run(mode, spec, cfg, output, delay=0., update='jacobi'):
    if mode not in ('centralized', 'distributed') or delay < 0:
        raise ValueError('Invalid mode or delay')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    planners = ([Planner(cfg, 2, spec.obstacles, spec.bounds)] if mode == 'centralized'
                else [Planner(cfg, 1, spec.obstacles, spec.bounds) for _ in range(2)])
    tracker = RouteTracker(spec, cfg)
    states, previous = spec.starts.copy(), np.zeros((2, 3))
    inbox = [hold_prediction(x, 0., cfg) for x in states]
    queue = []
    rows, arrivals = [], [None, None]
    collision_count, in_collision, min_distance = 0, False, float('inf')
    obstacle_collisions, in_obstacle_collision = 0, False
    history = [states.copy()]
    total_steps = int(round(cfg.max_time/cfg.dt))
    for tick in range(total_steps):
        now = tick*cfg.dt
        for delivery, m, prediction in sorted(queue, key=lambda z: z[0]):
            if delivery <= now+1e-9 and prediction.stamp >= inbox[m].stamp:
                inbox[m] = prediction
        queue = [item for item in queue if item[0] > now+1e-9]
        targets = np.array([tracker.target(m, states[m]) for m in range(2)])
        ages = [now-inbox[1-m].stamp for m in range(2)]
        if mode == 'centralized':
            plans = [planners[0].solve(states, targets, previous)]
            commands = plans[0].command.copy()
            predictions = [Prediction(now, cfg.dt, x) for x in plans[0].states]
        else:
            plans, predictions, commands = [], [], np.zeros((2, 3))
            snapshot = inbox.copy()
            for m in range(2):
                try:
                    peer, inflation = snapshot[1-m].align(now, cfg)
                    plan = planners[m].solve(states[m:m+1], targets[m:m+1], previous[m:m+1], peer, inflation)
                except ValueError as error:
                    plan = planners[m].stop(states[m:m+1], str(error))
                plans.append(plan)
                commands[m] = plan.command[0]
                prediction = Prediction(now, cfg.dt, plan.states[0])
                predictions.append(prediction)
                if update == 'gauss-seidel' and delay == 0:
                    snapshot[m] = prediction
        # Offline common safety supervisor. Its interventions are reported,
        # never attributed to MPC success or hidden from algorithm comparison.
        veto = not command_is_safe(states, commands, cfg, spec.obstacles, spec.bounds)
        if veto:
            commands[:] = 0.
            predictions = [hold_prediction(x, now, cfg) for x in states]
            for planner in planners:
                planner._warm = None
        for m, prediction in enumerate(predictions):
            queue.append((now+delay, m, prediction))
        next_states = np.array([step(x, u, cfg.dt) for x, u in zip(states, commands)])
        # Fine integration samples also check rotating/curved first-step motion.
        samples = [np.array([step(x, u, cfg.dt*k/10) for x, u in zip(states, commands)]) for k in range(11)]
        distance = min(segment_distance(a[0], b[0], a[1], b[1]) for a, b in zip(samples[:-1], samples[1:]))
        min_distance = min(min_distance, distance)
        collision = distance < 2*cfg.radius-1e-6
        collision_count += int(collision and not in_collision)
        in_collision = collision
        obstacle_collision = any(segment_distance(a[m], b[m], [ox, oy], [ox, oy]) < cfg.radius+radius-1e-6
                                 for a, b in zip(samples[:-1], samples[1:]) for m in range(2)
                                 for ox, oy, radius in spec.obstacles)
        obstacle_collisions += int(obstacle_collision and not in_obstacle_collision)
        in_obstacle_collision = obstacle_collision
        row = {'time': now+cfg.dt, 'min_distance': distance, 'collision': int(collision),
               'obstacle_collision': int(obstacle_collision),
               'safety_veto': int(veto), 'solver_failures': sum(not p.success for p in plans),
               'solver_seconds_total': sum(p.seconds for p in plans),
               'command_delta': float(np.linalg.norm(commands-previous))}
        for m in range(2):
            p = plans[0] if mode == 'centralized' else plans[m]
            row[f'robot_{m}_solver_seconds'] = p.seconds
            row[f'robot_{m}_status'] = p.status
            row[f'robot_{m}_peer_age'] = ages[m] if mode == 'distributed' else 0.
            for j, key in enumerate(('x', 'y', 'yaw')):
                row[f'robot_{m}_{key}'] = next_states[m, j]
            for j, key in enumerate(('vx', 'vy', 'wz')):
                row[f'robot_{m}_{key}'] = commands[m, j]
            if tracker.reached(m, next_states[m]) and arrivals[m] is None:
                arrivals[m] = now+cfg.dt
        rows.append(row)
        states, previous = next_states, commands.copy()
        history.append(states.copy())
        if all(tracker.reached(m, states[m]) for m in range(2)):
            break
    history = np.asarray(history)
    with (output/'timeseries.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    path_length = np.linalg.norm(np.diff(history[:, :, :2], axis=0), axis=2).sum(axis=0)
    window = max(2, int(5/cfg.dt))
    stalled = (len(history) > window and
               np.linalg.norm(history[-1, :, :2]-history[-window, :, :2], axis=1).max() < .05)
    reached = all(tracker.reached(m, states[m]) for m in range(2))
    summary = {'mode': mode, 'scenario': spec.name, 'delay_seconds': delay,
               'update': update if mode == 'distributed' else 'joint',
               'reached': reached, 'expected_reach': spec.expected_reach,
               'arrival_seconds': arrivals, 'duration_seconds': rows[-1]['time'],
               'collision_count': collision_count, 'obstacle_collision_count': obstacle_collisions,
               'min_distance': min_distance, 'required_distance': cfg.separation,
               'safety_violation_steps': sum(r['min_distance'] < cfg.separation-1e-5 for r in rows),
               'path_length': path_length.tolist(),
               'solver_seconds_mean': float(np.mean([r['solver_seconds_total'] for r in rows])),
               'solver_seconds_max': max(r['solver_seconds_total'] for r in rows),
               'solver_failures': sum(r['solver_failures'] for r in rows),
               'safety_vetoes': sum(r['safety_veto'] for r in rows),
               'command_delta_total': sum(r['command_delta'] for r in rows),
               'deadlock': bool(stalled and not reached), 'timeout': not reached,
               'final_goal_errors': np.linalg.norm(states[:, :2]-spec.goals[:, :2], axis=1).tolist(),
               'config': asdict(cfg)}
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    np.savez_compressed(output/'trajectory.npz', states=history, starts=spec.starts, goals=spec.goals)
    plot_run(history, rows, spec, cfg, summary, output)
    return summary


def plot_run(history, rows, spec, cfg, summary, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for m, color in enumerate(('tab:blue', 'tab:orange')):
        axes[0].plot(history[:, m, 0], history[:, m, 1], color=color, label=f'Robot {m}')
        axes[0].scatter(*spec.starts[m, :2], color=color, marker='o')
        axes[0].scatter(*spec.goals[m, :2], color=color, marker='*', s=130)
        for sample in np.linspace(0, len(history)-1, 8, dtype=int):
            axes[0].add_patch(Circle(history[sample, m, :2], cfg.radius, color=color, alpha=.08))
    for ox, oy, radius in spec.obstacles:
        axes[0].add_patch(Circle((ox, oy), radius, color='grey'))
    if spec.bounds:
        axes[0].axhline(spec.bounds[2], color='black')
        axes[0].axhline(spec.bounds[3], color='black')
    axes[0].set_aspect('equal')
    axes[0].set(xlabel='world x [m]', ylabel='world y [m]', title=f"{summary['mode']} / Scenario {spec.name}")
    axes[0].legend()
    times = [r['time'] for r in rows]
    axes[1].plot(times, [r['min_distance'] for r in rows])
    axes[1].axhline(cfg.separation, color='red', linestyle='--', label='Required separation')
    axes[1].set(xlabel='simulation time [s]', ylabel='center distance [m]')
    axes[1].legend()
    axes[2].plot(times, np.array([r['solver_seconds_total'] for r in rows])*1000)
    axes[2].axhline(cfg.dt*1000, color='red', linestyle='--', label='Control period')
    axes[2].set(xlabel='simulation time [s]', ylabel='total solver time [ms]')
    axes[2].legend()
    fig.tight_layout()
    fig.savefig(output/'overview.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=list('ABCD'), default='A')
    parser.add_argument('--mode', choices=['centralized', 'distributed', 'both'], default='both')
    parser.add_argument('--update', choices=['jacobi', 'gauss-seidel'], default='jacobi')
    parser.add_argument('--delay', type=float, default=0.)
    parser.add_argument('--config')
    parser.add_argument('--output', default='antbot_results')
    args = parser.parse_args()
    cfg, spec = MPCConfig.load(args.config), scenario(args.scenario)
    modes = ['centralized', 'distributed'] if args.mode == 'both' else [args.mode]
    summaries = []
    for mode in modes:
        dest = Path(args.output)/f'{args.scenario}_{mode}_delay{args.delay:g}_{args.update}'
        result = run(mode, spec, cfg, dest, args.delay if mode == 'distributed' else 0., args.update)
        summaries.append(result)
        print(json.dumps({k: result[k] for k in ['mode', 'scenario', 'reached', 'min_distance', 'collision_count', 'solver_failures', 'safety_vetoes', 'deadlock']}, ensure_ascii=False), flush=True)
    with (Path(args.output)/'comparison.csv').open('w', newline='') as stream:
        keys = ['mode', 'scenario', 'delay_seconds', 'reached', 'duration_seconds', 'min_distance', 'collision_count', 'obstacle_collision_count', 'solver_seconds_mean', 'solver_failures', 'safety_vetoes', 'deadlock']
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(summaries)
    # Scenario C is expected to stop; other baseline scenarios should reach.
    if any(r['collision_count'] or r['obstacle_collision_count'] or r['safety_violation_steps']
           or (spec.expected_reach and not r['reached'] and args.delay <= cfg.stale_timeout-cfg.dt)
           for r in summaries):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
