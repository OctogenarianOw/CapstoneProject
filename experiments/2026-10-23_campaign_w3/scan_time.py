#!/usr/bin/env python3
"""
Share of the scans fused with the pose at the scan time, per mission.

The P5 fix falls back to the latest pose when /clock or the pose
buffer cannot place a scan, counted in map.log but silent to the
mission. This campaign is the first flown with the fix: a mission
under 95 % flew on a map the fix did not make, and is flagged here.
The measure is that of 2026-10-21_p5_fix/analyse.py, reused.

    python3 scan_time.py [CAMPAIGN_DIR]
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / '2026-10-21_p5_fix'))
from analyse import scans_at_stamp  # noqa: E402

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent

shares = {}
for run in sorted((HERE / 'runs').iterdir()):
    if run.is_dir() and not run.name.startswith('.'):
        shares[run.name] = scans_at_stamp(run)
low = {k: v for k, v in shares.items() if not v > 0.95}
values = sorted(shares.values())
print(f'{len(shares)} missions, share of scans at the scan time: '
      f'min {values[0]:.1%}, median {values[len(values) // 2]:.1%}')
print(f'under 95 %: {len(low)}' + (f' {low}' if low else ''))
