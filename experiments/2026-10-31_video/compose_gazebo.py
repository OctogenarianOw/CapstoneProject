"""
The Gazebo film: one mission seen in the simulator itself, the camera
following the drone, as a separate short video (asked 1 Nov).

Role: the main cut shows RViz, the robot's own picture of the world.
This one shows the simulated world and the aircraft flying in it, once,
so a viewer sees what the map stands for. The Gazebo window is recorded
from the desktop (record.ps1 -Title '*Gazebo*'), cropped to its 3D view
and shown full frame, with a small panel drawn from the mission log in
a corner: mission time and state, energy left against what C1 needs.
Choice: a corner panel rather than the side-by-side chart of the main
cut, since this film is about the picture of the flight; the energy
story is told in the main cut.

    python3 compose_gazebo.py PLAN.json   (ROS and the workspace sourced)
"""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402

import compose as c1  # noqa: E402
import compose_v2 as c2  # noqa: E402
from compose import FPS, H, W, Mission, ffmpeg, recording_for, win  # noqa: E402

HERE = Path(__file__).resolve().parent
WORK = c1.SCRATCH / 'video/work_gz'
HW, HH = 775, 295                     # corner panel, pixels


def hud_frames(m, t0, t1, speed, out_dir, title):
    """Transparent corner panels, one per output frame."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in out_dir.glob('*.png'):
        f.unlink()
    budget = (m.cycles[0]['remaining_j'] + m.cycles[0]['consumed_j']) / 1e3
    fig = plt.figure(figsize=(HW / 100, HH / 100), dpi=100)
    fig.patch.set_alpha(0)
    fig.add_artist(FancyBboxPatch(
        (0.01, 0.02), 0.98, 0.96, boxstyle='round,pad=0,rounding_size=0.04',
        transform=fig.transFigure, color='#141413', alpha=0.78,
        zorder=-1))   # under the bar axes, not over them
    font = dict(family=c2.FONT)
    fig.text(0.05, 0.86, title, color='white', fontsize=19,
             fontweight='bold', va='center', **font)
    line1 = fig.text(0.05, 0.68, '', color='#e9e7e0', fontsize=17,
                     va='center', **font)
    line2 = fig.text(0.05, 0.50, '', color='#e9e7e0', fontsize=17,
                     va='center', **font)
    ax = fig.add_axes([0.05, 0.13, 0.90, 0.17])
    ax.set_xlim(0, budget)
    ax.set_ylim(0, 1)
    ax.axis('off')
    ax.add_patch(Rectangle((0, 0), budget, 1, color='#3a3935'))
    reserve = m.reserve
    ax.axvline(reserve, color='#a3a19a', lw=1.2)
    bar = Rectangle((0, 0), 0, 1, color=c2.COL['left'])
    ax.add_patch(bar)
    need_line = ax.axvline(0, color='white', lw=2.4)
    fig.text(0.05, 0.035, f'0', color='#a3a19a', fontsize=12, **font)
    fig.text(0.95, 0.035, f'{budget:.0f} kJ', color='#a3a19a', fontsize=12,
             ha='right', **font)
    fig.text(0.05 + 0.90 * reserve / budget, 0.035, 'reserve', ha='center',
             color='#a3a19a', fontsize=12, **font)
    n = int(round((t1 - t0) / speed * FPS))
    for k in range(n + 1):
        t = t0 + k * speed / FPS
        c = m.at(t)
        left = c['remaining_j'] / 1e3
        mg = c.get('i1_margin_j')
        state = c['state'].lower().replace('_', ' ')
        nz = sum(1 for z in c['zones'] if z['state'] == 'CONFIRMED'
                 and not z['provisional'])
        line1.set_text(f't = {max(t - m.t_takeoff, 0):5.1f} s   {state}   '
                       f'confirmed zones {nz}')
        line2.set_text(f'energy left {left:5.1f} kJ   I1 margin '
                       + ('n/a' if mg is None else f'{mg / 1e3:+5.1f} kJ'))
        bar.set_width(max(left, 0))
        need = None if mg is None else left - mg / 1e3
        need_line.set_visible(need is not None)
        if need is not None:
            need_line.set_xdata([need, need])
        fig.savefig(out_dir / f'{k:05d}.png', transparent=True)
    plt.close(fig)
    return n + 1


def gz_segment(spec, out):
    m = Mission(HERE / 'runs' / spec['run'])
    mkv, v0 = recording_for(m)
    t0 = m.t_takeoff + spec.get('from', -2.0)
    t1 = m.t_landed + spec.get('tail', 3.0)
    t0 = max(t0, -v0 + 0.3)
    speed = spec['speed']
    frames = WORK / f'{spec["run"]}_hud'
    n = hud_frames(m, t0, t1, speed, frames, spec['hud_title'])
    dur = n / FPS
    cx, cy, cw, ch = spec['crop']
    graph = (f'[0:v]crop={cw}:{ch}:{cx}:{cy},setpts=(PTS-STARTPTS)/{speed},'
             f'fps={FPS},scale={W}:{H}:force_original_aspect_ratio=increase,'
             f'crop={W}:{H}[g];'
             f'[g][1:v]overlay=36:{H - HH - 36}:format=auto,'
             f'format=yuv420p[v]')
    ffmpeg('-ss', f'{v0 + t0:.3f}', '-t', f'{(t1 - t0) + 1:.3f}',
           '-i', win(mkv), '-framerate', str(FPS),
           '-i', win(frames / '%05d.png'),
           '-filter_complex', graph, '-map', '[v]', '-t', f'{dur:.3f}',
           '-r', str(FPS), '-c:v', 'libx264', '-preset', 'medium',
           '-crf', '19', win(out))
    print(f'{out.name}: {spec["run"]} in Gazebo, mission '
          f'{t0 - m.t_takeoff:.0f} to {t1 - m.t_takeoff:.0f} s at x{speed}, '
          f'{dur:.1f} s')


def main():
    plan = json.loads(Path(sys.argv[1]).read_text())
    WORK.mkdir(parents=True, exist_ok=True)
    parts = []                        # cards reuse c2's orbit clips
    prefix = plan.get('prefix', 'gz')
    for k, item in enumerate(plan['parts']):
        out = WORK / f'{prefix}_{k:02d}.mp4'
        if item['kind'] == 'card':
            c2.card(item, out)
        else:
            gz_segment(item, out)
        parts.append(out)
    lst = WORK / f'{prefix}_parts.txt'
    lst.write_text(''.join(f"file '{win(p)}'\n" for p in parts))
    final = c1.SCRATCH / 'video' / plan['output']
    ffmpeg('-f', 'concat', '-safe', '0', '-i', win(lst), '-c', 'copy',
           '-movflags', '+faststart', win(final))
    print('video', final)


if __name__ == '__main__':
    main()
