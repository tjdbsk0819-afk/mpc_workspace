"""Append timing runs to one planner/horizon/scenario JSON without log files."""
import json
from pathlib import Path


class TimingArchive:
    def __init__(self, directory, planner, prediction_seconds, scenario, ego, planner_config):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        self.path=directory/f'{planner}_예측{prediction_seconds:g}초_시나리오{scenario}.json'
        if self.path.exists():
            self.data=json.loads(self.path.read_text())
            if not isinstance(self.data.get('runs'),list):
                raise ValueError('Existing timing archive has an unsupported format')
        else:
            self.data=dict(runs=[])
        self.run=dict(summary=None,timing=[])
        self.data['runs'].append(self.run)

    def save(self, summary, samples):
        self.run['summary']={key:summary[key] for key in ('samples','solver_ms','planning_ms')}
        if 'planning_hz' in summary:self.run['summary']['planning_hz']=summary['planning_hz']
        self.run['timing']=[{key:row[key] for key in ('stamp','solver_ms','planning_ms','start_wall_sec','completion_wall_sec','accepted') if key in row} for row in samples]
        self.data.update(run_count=len(self.data['runs']),
                         timing_sample_count=sum(len(r.get('timing',[])) for r in self.data['runs']))
        temporary=self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.data,ensure_ascii=False,indent=2))
        temporary.replace(self.path)


class ArrivalGate:
    """Latch arrival after fresh goal-position/stopped odometry for 0.5 seconds."""
    def __init__(self):
        self.since=None; self.last_stamp=None; self.arrival_stamp=None

    def observe(self, distance, speed, stamp):
        import math
        if self.arrival_stamp is not None:
            return True
        if not all(math.isfinite(v) for v in (distance,speed,stamp)):
            self.since=None
            return False
        if self.last_stamp is not None:
            if stamp<=self.last_stamp:
                if stamp<self.last_stamp:self.since=None
                return False
            if stamp-self.last_stamp>.35:self.since=None
        self.last_stamp=stamp
        if distance>.2 or speed>.05:
            self.since=None
            return False
        if self.since is None:self.since=stamp
        if stamp-self.since>=.5-1e-9:
            self.arrival_stamp=stamp
            return True
        return False
