#!/usr/bin/env python3
"""
Orbit a running Gazebo GUI camera around a world and save one render per
step, for the world views of the video (compose_v2.py shows them beside
the card that describes each simulation).

Role: Dorian asked to see each world before RViz shows the map of it.
The camera circles the world centre at a fixed radius and height,
looking at the centre; at each step the CameraTracking plugin's
/gui/move_to/pose service moves it, the script waits until the camera
pose the GUI publishes (/gui/camera/pose) has reached the target, then
asks the Screenshot plugin for the render and renames the file to its
step number.
Choice: one render per step through the GUI's own services, as for F3,
rather than recording the screen while the camera turns: the steps are
exact and evenly spaced whatever the software renderer's frame rate,
and nothing else on the desktop can enter a frame.
Rejected: the VideoRecorder plugin with a moving camera. Under llvmpipe
it records at the renderer's rate, a few uneven frames a second.

    python3 orbit_capture.py OUT_DIR --radius R --height Z --frames N
"""
import argparse
import math
from pathlib import Path
import subprocess
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node


def quat(yaw, pitch):
    """Quaternion (w, x, y, z) of yaw about z then pitch about y."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return cy * cp, -sy * sp, cy * sp, sy * cp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--radius', type=float, required=True)
    ap.add_argument('--height', type=float, required=True)
    ap.add_argument('--frames', type=int, default=600)
    ap.add_argument('--first', type=int, default=0)
    ap.add_argument('--last', type=int, default=None)
    ap.add_argument('--start_deg', type=float, default=-135.0)
    ap.add_argument('--cx', type=float, default=0.0)
    ap.add_argument('--cy', type=float, default=0.0)
    ap.add_argument('--cz', type=float, default=0.0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    shots = out / 'raw'
    shots.mkdir(exist_ok=True)

    node = Node()
    cam = {'p': None}
    lock = threading.Lock()

    def on_pose(msg):
        with lock:
            cam['p'] = (msg.position.x, msg.position.y, msg.position.z)
    node.subscribe(Pose, '/gui/camera/pose', on_pose)

    last = a.frames if a.last is None else a.last
    t0 = time.time()
    for k in range(a.first, last):
        th = math.radians(a.start_deg) + 2 * math.pi * k / a.frames
        x = a.cx + a.radius * math.cos(th)
        y = a.cy + a.radius * math.sin(th)
        z = a.cz + a.height
        yaw = th + math.pi
        pitch = math.atan2(a.height, a.radius)
        req = GUICamera()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
        w, qx, qy, qz = quat(yaw, pitch)
        req.pose.orientation.w, req.pose.orientation.x = w, qx
        req.pose.orientation.y, req.pose.orientation.z = qy, qz
        # The GUI can take a minute to come up on a cold start: retry the
        # first move, then expect an answer every step.
        give_up = time.time() + (120.0 if k == a.first else 10.0)
        while True:
            ok, _ = node.request('/gui/move_to/pose', req, GUICamera,
                                 Boolean, 5000)
            if ok or time.time() > give_up:
                break
            time.sleep(2.0)
        if not ok:
            raise SystemExit(f'step {k}: /gui/move_to/pose did not answer')
        # Wait for the published camera pose to reach the target.
        deadline = time.time() + 5.0
        while time.time() < deadline:
            with lock:
                p = cam['p']
            if p and math.dist(p, (x, y, z)) < 0.01 * a.radius:
                break
            time.sleep(0.02)
        time.sleep(0.15)          # one more render at the final pose
        before = set(shots.iterdir())
        # Through the gz CLI, as f3_capture does: the same request from
        # the Python binding timed out on 1 Nov while the CLI answered in
        # 0.4 s.
        # On 1 Nov one request in 64 to 248 went unanswered (once per
        # world, at a different step each time): retry before giving up.
        for attempt in range(4):
            r = subprocess.run(
                ['gz', 'service', '-s', '/gui/screenshot', '--reqtype',
                 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '10000', '--req', f'data: "{shots}"'],
                capture_output=True, text=True)
            if 'true' in r.stdout:
                break
            print(f'step {k}: screenshot unanswered, retry {attempt + 1}',
                  flush=True)
            time.sleep(1.0)
        else:
            raise SystemExit(f'step {k}: /gui/screenshot did not answer')
        deadline = time.time() + 10.0
        new = []
        while time.time() < deadline and not new:
            new = [f for f in set(shots.iterdir()) - before
                   if f.suffix == '.png']
            time.sleep(0.05)
        if not new:
            raise SystemExit(f'step {k}: no screenshot written')
        time.sleep(0.1)           # let the writer finish the file
        new[0].rename(out / f'{k:04d}.png')
        if k % 25 == 0:
            rate = (k - a.first + 1) / (time.time() - t0)
            print(f'step {k}/{last}, {rate:.2f} steps/s', flush=True)
    print(f'done, {last - a.first} steps in {time.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
