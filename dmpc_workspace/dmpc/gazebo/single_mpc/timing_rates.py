"""Planning rates measured from monotonic event timestamps, not inverse solve time."""
import math
import statistics


def _interval_rates(events):
    gaps=[b-a for a,b in zip(events,events[1:]) if b>a]
    if not gaps:
        return dict(effective_hz=None,latest_hz=None,min_hz=None,max_hz=None,median_hz=None,intervals=0)
    rates=[1./gap for gap in gaps]
    return dict(effective_hz=len(gaps)/sum(gaps),latest_hz=rates[-1],min_hz=min(rates),
                max_hz=max(rates),median_hz=statistics.median(rates),intervals=len(gaps))


def planning_rates(samples, configured_hz):
    starts=[s['start_wall_sec'] for s in samples if isinstance(s.get('start_wall_sec'),(float,int)) and math.isfinite(s['start_wall_sec'])]
    completions=[s for s in samples if isinstance(s.get('completion_wall_sec'),(float,int)) and math.isfinite(s['completion_wall_sec'])]
    ends=[s['completion_wall_sec'] for s in completions]
    valid_hz=None
    if len(ends)>1 and ends[-1]>ends[0]:
        # Count accepted completion events in (first completion, last completion].
        valid_hz=sum(bool(s.get('accepted',False)) for s in completions[1:])/(ends[-1]-ends[0])
    return dict(configured_hz=configured_hz,start=_interval_rates(starts),completion=_interval_rates(ends),
                valid_plan_hz=valid_hz,clock='monotonic_wall')


def format_rates(rates):
    def number(value):return '--' if value is None else f'{value:.2f}'
    return (f"planning Hz: target={rates['configured_hz']:g}, "
            f"start={number(rates['start']['effective_hz'])}, "
            f"completion={number(rates['completion']['effective_hz'])}, "
            f"valid_plan={number(rates['valid_plan_hz'])}")
