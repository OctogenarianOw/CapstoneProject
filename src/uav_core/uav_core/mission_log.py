"""
Mission log, one timestamped directory per run (spec S17).

Role: make every figure of the thesis regenerable from a directory,
without running the simulation again, and every decision explainable
after the fact.

Layout of <root>/<YYYY-mm-dd_HH-MM-SS>/:
  config.json    every parameter of the run, git HEAD and status
  cycles.jsonl   one line per decision cycle, every candidate with its
                 numbers, kept or not, not only the winner (S14)
  events.jsonl   one line per event: state change, zone transition,
                 blacklisting, replanning, landing commitment
  maps/          map snapshots, map_<seconds>.npz, every snapshot_s

Choice: JSON lines, append-only and flushed at each line, so a crash
loses at most the line being written and any tool reads them back.
Rejected: rosbag as the primary record. It needs ROS to read, and it
stores messages, not the decision variables C1 and C2 compute.
"""

import json
import math
import os
import subprocess
import time

import numpy as np


def _clean(value):
    """Make numpy values and non-finite floats JSON-safe."""
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


class MissionLog:
    """Append-only JSON lines plus map snapshots in one directory."""

    def __init__(self, root, config, repo=None, snapshot_s=10.0):
        stamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.dir = os.path.join(os.path.expanduser(root), stamp)
        os.makedirs(os.path.join(self.dir, 'maps'), exist_ok=True)
        self.snapshot_s = snapshot_s
        self.last_snapshot = -math.inf
        config = dict(config)
        if repo is not None:
            config['git_head'] = self._git(repo, 'rev-parse', 'HEAD')
            config['git_status'] = self._git(repo, 'status', '--short')
        with open(os.path.join(self.dir, 'config.json'), 'w') as f:
            json.dump(_clean(config), f, indent=1)
            f.write('\n')
        self._cycles = open(os.path.join(self.dir, 'cycles.jsonl'), 'a')
        self._events = open(os.path.join(self.dir, 'events.jsonl'), 'a')

    @staticmethod
    def _git(repo, *args):
        try:
            return subprocess.run(['git', '-C', repo, *args],
                                  capture_output=True, text=True,
                                  timeout=5).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return 'unavailable'

    def cycle(self, t, record):
        """Append one decision cycle."""
        self._write(self._cycles, dict(t=t, **record))

    def event(self, t, kind, **data):
        """Append one event."""
        self._write(self._events, dict(t=t, kind=kind, **data))

    def maybe_snapshot(self, t, **layers):
        """Save the map layers if snapshot_s has passed since the last."""
        if t - self.last_snapshot < self.snapshot_s:
            return
        self.last_snapshot = t
        np.savez_compressed(os.path.join(self.dir, 'maps',
                                         f'map_{t:08.1f}.npz'), **layers)

    @staticmethod
    def _write(stream, record):
        stream.write(json.dumps(_clean(record)) + '\n')
        stream.flush()

    def close(self):
        """Close both line files."""
        self._cycles.close()
        self._events.close()
