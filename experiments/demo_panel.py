#!/usr/bin/env python3
"""
Demonstration panel: set a mission up, start it, then steer it in flight.

Role: one window for the live demonstrations of the thesis. Before the
start it sets what the mission is launched with: world, battery budget,
policy, alpha, reserve, session length. Start runs experiments/demo.sh
with these values (PX4, Gazebo, RViz and the ROS nodes, logged like any
measurement under experiments/demos/). Once started, the same window
changes alpha and the reserve in flight, sends the return and land
command, ends the session, and shows the mission state, the energy left
and the I1 margin.

Choices:
  - a panel of its own instead of rqt_reconfigure. rqt lists every node
    and every parameter, and its filter box reads as an input field: on
    22 Oct its first use typed the alpha value into the filter. The
    panel shows only what acts, in the order of a mission.
  - the battery is fixed once the mission starts. It is the energy
    budget of the mission (spec S10 note); a battery does not change in
    flight, and C1 is about what the aircraft can still do with it.
  - alpha and the reserve change in flight through the parameter service
    of exploration_manager, the path of the scripted changes of
    2026-10-20_alpha_live: applied and logged as those were. The manager
    refuses any other parameter in flight.
  - the state shown is read from the mission's own cycles.jsonl, the
    file every result is computed from, not from extra topics: what the
    panel shows is what the thesis reports.
Rejected: Tk, for PyQt5, already installed for RViz and rqt, whose
sliders and layouts read well on a projector; launch arguments alone,
which give no control in flight.

    bash experiments/demo_panel.sh     # sources ROS and the workspace
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty

EXP = Path(__file__).resolve().parent
DEMO = EXP / 'demo.sh'
# Shown name, demo.sh world, default budget kJ, default alpha: the
# settings each world was measured with (demo.sh header).
WORLDS = [('W1, validation world (100 m)', 'w1', 16, 0.50),
          ('W2, rare landing zones (200 m)', 'w2', 48, 0.50),
          ('W3, road, town, forest, rock (240 m)', 'w3', 80, 0.75)]
POLICIES = [('C1 + C2, the proposed method', 'c1_c2'),
            ('B1, information gain, home at 30 % battery', 'gain'),
            ('B0, nearest frontier, no energy constraint', 'nearest')]
STATES = {'INIT': 'waiting for PX4', 'TAKEOFF': 'taking off',
          'BOOTSTRAP': 'first maps', 'SELECTING': 'choosing a frontier',
          'NAVIGATING': 'flying to a frontier',
          'RETURNING': 'returning to a landing zone',
          'COMMIT_LANDING': 'landing', 'LANDED': 'landed',
          'EMERGENCY': 'emergency landing'}
ALPHA_STEP = 0.05


def last_cycle(mission_root):
    """Last complete record of the mission's cycles.jsonl, or None."""
    files = sorted(Path(mission_root).glob('*/cycles.jsonl'))
    if not files:
        return None
    with open(files[-1], 'rb') as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        f.seek(max(0, size - 400_000))
        lines = [x for x in f.read().split(b'\n') if x.strip()]
    for line in reversed(lines[-2:]):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None


class Panel(QtWidgets.QWidget):
    """Mission set-up, start, live parameters and state."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.client = AsyncParameterClient(node, 'exploration_manager')
        self.stop_pub = node.create_publisher(Empty, '/mission/stop', 10)
        self.proc = None
        self.out = None
        self.pending = {}
        self.setWindowTitle('UAV demonstration')
        self.build()
        self.on_world(0)
        self.on_policy(0)
        self.set_running(False)

        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.node, timeout_sec=0.0))
        self.ros_timer.start(50)
        self.state_timer = QtCore.QTimer(self)
        self.state_timer.timeout.connect(self.refresh)
        self.state_timer.start(1000)
        self.send_timer = QtCore.QTimer(self)
        self.send_timer.setSingleShot(True)
        self.send_timer.timeout.connect(self.send)

    # ------------------------------------------------------------ layout
    def build(self):
        top = QtWidgets.QVBoxLayout(self)

        setup = QtWidgets.QGroupBox('1. Mission, set before the start')
        form = QtWidgets.QFormLayout(setup)
        self.world = QtWidgets.QComboBox()
        self.world.addItems([w[0] for w in WORLDS])
        self.world.currentIndexChanged.connect(self.on_world)
        self.battery = QtWidgets.QSpinBox()
        self.battery.setRange(4, 400)
        self.battery.setSuffix(' kJ')
        self.battery.setToolTip('Energy budget of the mission, rotor shaft '
                                'energy (spec S10). Fixed once started.')
        self.battery.valueChanged.connect(self.show_reserve)
        self.policy = QtWidgets.QComboBox()
        self.policy.addItems([p[0] for p in POLICIES])
        self.policy.currentIndexChanged.connect(self.on_policy)
        self.minutes = QtWidgets.QSpinBox()
        self.minutes.setRange(2, 60)
        self.minutes.setValue(15)
        self.minutes.setSuffix(' min')
        self.minutes.setToolTip('The windows close after this time, or on '
                                'End session.')
        form.addRow('World', self.world)
        form.addRow('Battery', self.battery)
        form.addRow('Policy', self.policy)
        form.addRow('Session length', self.minutes)
        self.start_btn = QtWidgets.QPushButton('Start')
        self.start_btn.setStyleSheet('font-weight: bold; padding: 6px')
        self.start_btn.clicked.connect(self.start)
        form.addRow(self.start_btn)
        top.addWidget(setup)

        live = QtWidgets.QGroupBox('2. Before the start and in flight')
        grid = QtWidgets.QGridLayout(live)
        self.alpha = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.alpha.setRange(0, round(1.0 / ALPHA_STEP))
        self.alpha.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.alpha.setTickInterval(5)
        self.alpha.valueChanged.connect(self.on_alpha)
        self.alpha_val = QtWidgets.QLabel()
        self.alpha_val.setMinimumWidth(150)
        self.reserve = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.reserve.setRange(0, 50)
        self.reserve.setValue(10)
        self.reserve.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.reserve.setTickInterval(10)
        self.reserve.valueChanged.connect(self.on_reserve)
        self.reserve_val = QtWidgets.QLabel()
        grid.addWidget(QtWidgets.QLabel('alpha'), 0, 0)
        grid.addWidget(self.alpha, 0, 1)
        grid.addWidget(self.alpha_val, 0, 2)
        hint = QtWidgets.QLabel('0 = keep landing zones close, '
                                '1 = information gain only; acts at the '
                                'next frontier choice')
        hint.setStyleSheet('color: gray')
        grid.addWidget(hint, 1, 1, 1, 2)
        grid.addWidget(QtWidgets.QLabel('reserve'), 2, 0)
        grid.addWidget(self.reserve, 2, 1)
        grid.addWidget(self.reserve_val, 2, 2)
        hint = QtWidgets.QLabel('energy C1 always keeps for landing; raising '
                                'it can send the aircraft home at once')
        hint.setStyleSheet('color: gray')
        grid.addWidget(hint, 3, 1, 1, 2)
        self.sent = QtWidgets.QLabel('')
        self.sent.setWordWrap(True)
        grid.addWidget(self.sent, 4, 0, 1, 3)
        buttons = QtWidgets.QHBoxLayout()
        self.home_btn = QtWidgets.QPushButton('Return and land')
        self.home_btn.setToolTip('The aircraft returns to a confirmed '
                                 'landing zone and lands (S15.4).')
        self.home_btn.clicked.connect(self.go_home)
        self.end_btn = QtWidgets.QPushButton('End session')
        self.end_btn.setToolTip('Closes PX4, Gazebo and RViz; the logs stay '
                                'in experiments/demos/.')
        self.end_btn.clicked.connect(self.end_session)
        buttons.addWidget(self.home_btn)
        buttons.addWidget(self.end_btn)
        grid.addLayout(buttons, 5, 0, 1, 3)
        top.addWidget(live)

        state = QtWidgets.QGroupBox('3. Mission state, from the mission log')
        sform = QtWidgets.QFormLayout(state)
        self.fields = {}
        for key in ('session', 'state', 'time', 'energy', 'margin', 'zones',
                    'seen'):
            self.fields[key] = QtWidgets.QLabel('')
        sform.addRow('Session', self.fields['session'])
        sform.addRow('State', self.fields['state'])
        sform.addRow('Mission time', self.fields['time'])
        sform.addRow('Energy left', self.fields['energy'])
        sform.addRow('I1 margin', self.fields['margin'])
        sform.addRow('Confirmed zones', self.fields['zones'])
        sform.addRow('Area seen', self.fields['seen'])
        top.addWidget(state)

    # ---------------------------------------------------------- set-up
    def on_world(self, i):
        _, _, budget, alpha = WORLDS[i]
        self.battery.setValue(budget)
        self.alpha.setValue(round(alpha / ALPHA_STEP))

    def on_policy(self, i):
        c2 = POLICIES[i][1] == 'c1_c2'
        self.alpha.setEnabled(c2)
        self.alpha.setToolTip('' if c2 else 'alpha weights C2, used by '
                              'C1 + C2 only')
        self.on_alpha(self.alpha.value())

    def alpha_value(self):
        return round(self.alpha.value() * ALPHA_STEP, 2)

    def reserve_value(self):
        return round(self.reserve.value() / 100.0, 2)

    def on_alpha(self, _v):
        if POLICIES[self.policy.currentIndex()][1] != 'c1_c2':
            self.alpha_val.setText('not used by this policy')
            return
        self.alpha_val.setText(f'{self.alpha_value():.2f}')
        self.queue('alpha', self.alpha_value())

    def on_reserve(self, _v):
        self.show_reserve()
        self.queue('reserve_fraction', self.reserve_value())

    def show_reserve(self):
        r = self.reserve_value()
        self.reserve_val.setText(f'{100 * r:.0f} % '
                                 f'({r * self.battery.value():.1f} kJ)')

    # ------------------------------------------------------- session
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def set_running(self, on):
        for w in (self.world, self.battery, self.policy, self.minutes,
                  self.start_btn):
            w.setEnabled(not on)
        self.home_btn.setEnabled(on)
        self.end_btn.setEnabled(on)

    def start(self):
        key = WORLDS[self.world.currentIndex()][1]
        policy = POLICIES[self.policy.currentIndex()][1]
        stamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.out = EXP / 'demos' / f'{stamp}_{key}'
        self.out.mkdir(parents=True)
        env = dict(os.environ, POLICY=policy,
                   RESERVE=f'{self.reserve_value():.2f}', SLIDER='false',
                   OUT=str(self.out))
        log = open(self.out / 'demo.out', 'w')
        self.proc = subprocess.Popen(
            ['bash', str(DEMO), key, f'{self.alpha_value():.2f}',
             str(self.battery.value()), str(60 * self.minutes.value())],
            env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.pending.clear()
        self.sent.setText('')
        self.set_running(True)
        self.fields['session'].setText(f'started, logs in {self.out}')

    def queue(self, name, value):
        """Send a change once the slider rests, if a mission is running."""
        if not self.running():
            return
        self.pending[name] = value
        self.send_timer.start(300)

    def send(self):
        if not self.pending or not self.running():
            return
        if not self.client.services_are_ready():
            self.sent.setText('manager not up yet: the change will apply '
                              'as soon as it is')
            self.send_timer.start(1000)
            return
        sent = dict(self.pending)
        self.pending.clear()
        future = self.client.set_parameters(
            [Parameter(n, value=float(v)) for n, v in sent.items()])
        future.add_done_callback(lambda f: self.on_sent(f, sent))

    def on_sent(self, future, sent):
        try:
            results = future.result().results
        except Exception as e:      # noqa: BLE001, shown, not raised
            self.sent.setText(f'change not applied: {e}')
            return
        text = []
        for (name, value), r in zip(sent.items(), results):
            label = 'alpha' if name == 'alpha' else 'reserve'
            text.append(f'{label} {value:.2f} applied' if r.successful
                        else f'{label} refused: {r.reason}')
        self.sent.setText('; '.join(text) + time.strftime(' at %H:%M:%S'))

    def go_home(self):
        self.stop_pub.publish(Empty())
        self.sent.setText('return and land requested'
                          + time.strftime(' at %H:%M:%S'))

    def end_session(self):
        if self.out is not None:
            (self.out / 'stop').touch()
        self.fields['session'].setText('ending: closing PX4, Gazebo and '
                                       'RViz')

    # --------------------------------------------------------- state
    def refresh(self):
        if self.proc is None:
            if self.out is None:
                self.fields['session'].setText('not started')
            return
        if self.proc.poll() is not None:
            self.proc = None
            self.set_running(False)
            self.fields['session'].setText(f'ended, logs in {self.out}')
            return
        c = last_cycle(self.out / 'missions')
        if c is None:
            self.fields['state'].setText('starting PX4, the simulator and '
                                         'the ROS nodes (about 40 s)')
            return
        budget = self.battery.value()
        self.fields['state'].setText(STATES.get(c['state'], c['state']))
        self.fields['time'].setText(f'{c["t"]:.0f} s')
        left = c['remaining_j']
        if left is not None:
            self.fields['energy'].setText(f'{left / 1000:.1f} kJ of '
                                          f'{budget} kJ')
        m = c['i1_margin_j']
        if m is None:
            self.fields['margin'].setText('n/a')
        else:
            colour = 'red' if m < 0 else 'black'
            self.fields['margin'].setText(
                f'<span style="color:{colour}">{m / 1000:+.2f} kJ</span>'
                ' beyond what the return and the reserve need')
        conf = [z for z in c['zones'] if z['state'] == 'CONFIRMED'
                and not z.get('provisional')]
        self.fields['zones'].setText(str(len(conf)))
        area = c['seen_in_area'] + c['unknown_in_area']
        if area:
            self.fields['seen'].setText(
                f'{100 * c["seen_in_area"] / area:.0f} %')

    def closeEvent(self, event):    # noqa: N802, Qt name
        if self.running():
            self.end_session()
        event.accept()


def main():
    # rclpy's own handlers only shut its context down, and the Qt loop
    # then runs on, raising at every spin (22 Oct): keep the defaults,
    # so Ctrl+C and kill close the panel.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    node = rclpy.create_node('demo_panel')
    app = QtWidgets.QApplication(sys.argv)
    font = app.font()
    font.setPointSize(11)
    app.setFont(font)
    panel = Panel(node)
    panel.resize(620, 640)
    panel.show()
    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
