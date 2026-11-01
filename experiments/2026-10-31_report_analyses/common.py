"""
Shared loader for the report analyses of 31 Oct 2026.

Role: read the mission logs of the three final campaigns (210 missions)
and of the provoked S9.5 experiment (13 missions), and reduce each
mission to the fields the analyses need, cached once per mission.

Design choice: a compact per-mission cache (gzip JSON under cache/,
ignored by git) built from cycles.jsonl and events.jsonl. The raw
cycles weigh about 8 MB per mission and hold every frontier of every
cycle; parsing them once and keeping only the candidates, zones and
energy terms makes every analysis script run in seconds and read the
same numbers. Rejected: one script per analysis parsing the raw logs,
which repeats a 10 minute parse and lets two scripts disagree on how a
field was read. Nothing here recomputes a decision: every number is the
one the exploration manager logged in flight.
"""
import gzip
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = HERE.parent
CACHE = HERE / 'cache'

CAMPAIGNS = {
    'W1': EXP / '2026-10-23_campaign_w1_final',
    'W2': EXP / '2026-10-23_campaign_w2_final',
    'W3': EXP / '2026-10-23_campaign_w3_final',
}
S95 = EXP / '2026-10-24_s95_return'
BUDGET_KJ = {'W1': 16.0, 'W2': 48.0, 'W3': 80.0}
CONFIGS = ['B0', 'B1', 'B3a000', 'B3a025', 'B3a050', 'B3a075', 'B3a100']
ALPHAS = {'B3a000': 0.0, 'B3a025': 0.25, 'B3a050': 0.5, 'B3a075': 0.75,
          'B3a100': 1.0}
LABEL = {'B0': 'B0', 'B1': 'B1', 'B3a000': 'α=0', 'B3a025': 'α=0.25',
         'B3a050': 'α=0.5', 'B3a075': 'α=0.75', 'B3a100': 'α=1'}


def mission_dir(run_dir):
    """The single timestamped mission directory of a run."""
    subs = sorted(p for p in (run_dir / 'missions').iterdir() if p.is_dir())
    if len(subs) != 1:
        raise RuntimeError(f'{run_dir}: {len(subs)} mission directories')
    return subs[0]


def runs(world):
    """(config, run number, run directory) of a final campaign, sorted."""
    out = []
    for d in sorted((CAMPAIGNS[world] / 'runs').iterdir()):
        m = re.match(r'(B0|B1|B3a\d{3})_r(\d+)$', d.name)
        if m:
            out.append((m.group(1), int(m.group(2)), d))
    return out


def _compact_cycle(c):
    cands = [{k: x.get(k) for k in ('gain', 'path_dist_m', 'e_go', 'e_return',
                                     'e_total', 'zone_id', 'admissible', 'g',
                                     'm', 'm_share', 'u', 'chosen')}
             for x in c.get('candidates') or []]
    zones = [{k: z.get(k) for k in ('id', 'state', 'provisional',
                                     'touchdown_ne', 'area_m2', 'landing_m2',
                                     'passes')}
             for z in c.get('zones') or []]
    timings = c.get('timings_ms') or {}
    return {
        't': c['t'], 'state': c['state'], 'ned': c.get('vehicle_ned'),
        'remaining_j': c.get('remaining_j'), 'consumed_j': c.get('consumed_j'),
        'margin_j': c.get('i1_margin_j'), 'violations': c.get('violations'),
        'overhead_s': c.get('overhead_s'), 'seen': c.get('seen_in_area'),
        'unknown': c.get('unknown_in_area'),
        'landing_cells': c.get('landing_cells'),
        'n_frontiers': len(c.get('frontiers') or []),
        'n_rejected_frontiers': sum(1 for f in c.get('frontiers') or []
                                    if f.get('rejected')),
        'zones': zones, 'candidates': cands, 'decision': c.get('decision'),
        'timings_ms': timings, 'compute_ms': float(sum(timings.values())),
    }


def load(run_dir):
    """Compact record of one mission, built on first use and cached."""
    CACHE.mkdir(exist_ok=True)
    key = f'{run_dir.parent.parent.name}__{run_dir.name}.json.gz'
    path = CACHE / key
    if path.exists():
        with gzip.open(path, 'rt') as f:
            return json.load(f)
    md = mission_dir(run_dir)
    cycles = []
    with open(md / 'cycles.jsonl') as f:
        for line in f:
            cycles.append(_compact_cycle(json.loads(line)))
    with open(md / 'events.jsonl') as f:
        events = [json.loads(line) for line in f]
    config = json.loads((md / 'config.json').read_text())
    rec = {'run': run_dir.name, 'mission_dir': str(md), 'config': config,
           'cycles': cycles, 'events': events}
    with gzip.open(path, 'wt') as f:
        json.dump(rec, f)
    return rec


def missions_csv(world):
    """Rows of the campaign's missions.csv, keyed by run name."""
    import csv
    rows = {}
    with open(CAMPAIGNS[world] / 'missions.csv') as f:
        for r in csv.DictReader(f):
            rows[f"{r['config']}_r{r['run']}"] = r
    return rows


def median_iqr(x):
    import numpy as np
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float('nan'), float('nan'), float('nan')
    q1, med, q3 = np.percentile(x, [25, 50, 75])
    return float(med), float(q1), float(q3)


def fmt(x, nd=2):
    med, q1, q3 = median_iqr(x)
    return f'{med:.{nd}f} [{q1:.{nd}f}, {q3:.{nd}f}]'
