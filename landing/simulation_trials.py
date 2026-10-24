"""Repeatable closed-loop experiments; results apply only to this planar model."""
import math
from pathlib import Path
import json

from .simulation import LandingSimulation


def run_trial(simulation, flight, *, gps=1.2, wind=.5, vision=True, dropout=False,
              obscure_return=False, max_time=120.):
    simulation.start(flight)
    dropout_start = None
    reacquired = False
    while simulation.time < max_time and simulation.phase != 'LANDED':
        if dropout and dropout_start is None and simulation.phase == 'VISUAL APPROACH':
            dropout_start = simulation.time
        hidden = obscure_return and simulation.phase not in ('TAKEOFF', 'OUTBOUND')
        if dropout_start is not None:
            hidden |= simulation.time < dropout_start+1.2
            if simulation.time >= dropout_start+1.2 and simulation.phase == 'VISUAL APPROACH':
                reacquired = True
        simulation.advance(gps=gps, wind=wind, vision=vision, occluded=hidden)
    state = simulation.snapshot()
    landed = state['phase'] == 'LANDED'
    error = state['error']
    metrics = state['metrics']
    success = landed and error <= .15 and not metrics['false_locks'] and not metrics['inaccurate_locks']
    reason = 'landed_within_tolerance' if success else (
        'landed_outside_tolerance' if landed else
        'no_return_acquisition' if metrics['first_lock_s'] is None else 'tracking_or_guidance_timeout')
    return dict(flight=state['flight'], gps_error_m=gps, wind=wind, vision=vision,
                landed=landed, success=success, outcome=reason, final_phase=state['phase'],
                landing_error_m=error, simulated_time_s=state['time'], references=state['references'],
                mapping_status=state['mapping'], dropout_injected=dropout_start is not None,
                recovered_after_dropout=reacquired, metrics=metrics)


def run_trials(seeds=(7, 42), output=None, max_time=120.):
    if not math.isfinite(max_time) or not 10 <= max_time <= 600:
        raise ValueError('Trial duration must be between 10 and 600 simulated seconds')
    seeds = list(seeds)
    if not seeds or len(seeds) > 20 or any(not isinstance(seed, int) for seed in seeds):
        raise ValueError('Supply between 1 and 20 integer terrain seeds')
    baseline = dict(altitude=4., distance=8., return_yaw=0., tilt=0.)
    rotated = dict(altitude=6., distance=20., return_yaw=90., tilt=8.)
    cases = [
        ('baseline', baseline, {}),
        ('rotated_tilted', rotated, {'wind': 1.}),
        ('visual_loss_recovery', rotated, {'dropout': True}),
        ('gps_only', baseline, {'vision': False}),
        ('ground_obscured_on_return', baseline, {'obscure_return': True}),
    ]
    results = []
    for seed in seeds:
        simulation = LandingSimulation(seed=seed)
        for name, flight, settings in cases:
            result = run_trial(simulation, flight, max_time=max_time, **settings)
            results.append(dict(seed=seed, scenario=name, **result))
    report = dict(kind='closed_loop_planar_simulation', tolerance_m=.15, max_time_s=max_time,
                  limitation='Ideal telemetry, flat procedural ground, kinematic motion; not Betaflight or real flight validation.',
                  runs=results, summary=dict(runs=len(results), landed=sum(r['landed'] for r in results),
                  within_tolerance=sum(r['success'] for r in results),
                  timeouts=sum(not r['landed'] for r in results),
                  false_locks=sum(r['metrics']['false_locks'] for r in results),
                  inaccurate_locks=sum(r['metrics']['inaccurate_locks'] for r in results)))
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report
