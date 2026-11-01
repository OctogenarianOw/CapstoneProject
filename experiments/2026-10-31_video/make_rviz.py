"""
Write one RViz configuration per world for the filmed missions.

Role: the demonstration configuration (install/.../exploration.rviz) is
framed for W1 and shows the display panel. For a video the 3D view must
fill the window and frame the explored area of each world. This copies
it and changes only the view: left dock hidden, orbit distance and pitch
per world, frame rate lowered to 15 Hz (the window renders in software
under WSLg, and every frame it draws is CPU taken from the simulation).

    python3 make_rviz.py     # writes rviz/validation.rviz, w2.rviz, w3.rviz
"""
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
SRC = (HERE.parents[1] / 'install' / 'uav_core' / 'share' / 'uav_core'
       / 'rviz' / 'exploration.rviz')
VIEWS = {'validation': (78.0, 0.80), 'w2': (150.0, 0.85),
         'w3': (185.0, 0.85)}


def main():
    text = SRC.read_text()
    out = HERE / 'rviz'
    out.mkdir(exist_ok=True)
    for world, (distance, pitch) in VIEWS.items():
        t = re.sub(r'(\n      Distance: )[0-9.]+', rf'\g<1>{distance}', text)
        t = re.sub(r'(\n      Pitch: )[0-9.]+', rf'\g<1>{pitch}', t)
        t = t.replace('Hide Left Dock: false', 'Hide Left Dock: true')
        t = t.replace('    Frame Rate: 30', '    Frame Rate: 15')
        (out / f'{world}.rviz').write_text(t)
        print(out / f'{world}.rviz', distance, pitch)


if __name__ == '__main__':
    main()
