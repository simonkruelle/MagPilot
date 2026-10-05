#!/usr/bin/env python3
"""Observe a Gazebo flange reaching targets while an existing interface drives it.

This tool publishes no robot commands. Start the launcher in Simulation mode,
then Robot, Arm nodes and Interface, and open Virtual task. A measured run needs
live Gazebo /clock and TF feedback; --dry-run only writes a reproducible plan.
"""

import argparse
import math
import os
import re
import signal
import sys
import time
from datetime import datetime

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from colmag.target_reaching import (  # noqa: E402
    PilotRun, TargetTrial, TrialSettings, default_targets, distance,
    require_simulation,
)


def safe_component(value):
    value = re.sub(r'[^A-Za-z0-9_.-]+', '_', value.strip()).strip('._-')
    if not value:
        raise ValueError('Enter a nonempty run ID.')
    return value


class FeedbackUnavailable(RuntimeError):
    pass


class RosFeedback:
    """Read actual robot-state TF and require an advancing simulation clock."""

    def __init__(self, base_frame, ee_frame, max_feedback_gap_s=0.25):
        import rosnode
        import rospy
        import tf2_ros
        from rosgraph_msgs.msg import Clock

        self.rospy, self.rosnode, self.tf2_ros = rospy, rosnode, tf2_ros
        # Check the master before init_node, which otherwise keeps retrying a
        # missing master instead of returning an actionable start error.
        self.verify_simulation()
        rospy.init_node('colmag_target_reaching_pilot', anonymous=True, disable_signals=True)
        self.base_frame, self.ee_frame = base_frame, ee_frame
        self.max_feedback_gap_s = max_feedback_gap_s
        self.tf_stamp_s = None
        self.tf_changed_wall_s = 0.0
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.clock_s = None
        self.clock_changed_wall_s = 0.0
        self.clock_subscriber = rospy.Subscriber('/clock', Clock, self._clock, queue_size=1)

    def verify_simulation(self):
        try:
            require_simulation(self.rospy.get_param('/use_sim_time', False),
                               self.rosnode.get_node_names())
        except Exception as exc:
            raise FeedbackUnavailable(str(exc))

    def _clock(self, message):
        stamp = message.clock.to_sec()
        if stamp != self.clock_s:
            self.clock_changed_wall_s = time.monotonic()
        self.clock_s = stamp

    def sample(self):
        wall_s = time.monotonic()
        if self.clock_s is None or wall_s - self.clock_changed_wall_s > 0.75:
            raise FeedbackUnavailable('Gazebo clock is paused or unavailable.')
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, self.ee_frame, self.rospy.Time(0))
        except self.tf2_ros.TransformException as exc:
            raise FeedbackUnavailable('Waiting for measured flange TF: {}'.format(exc))
        # Robot state publisher transforms must advance with Gazebo. A valid
        # command pose or stale TF must never stand in for the measured flange.
        age = self.clock_s - transform.header.stamp.to_sec()
        if age < -0.1 or age > 0.25:
            raise FeedbackUnavailable('Measured flange TF is stale ({:.2f} s).'.format(age))
        stamp = transform.header.stamp.to_sec()
        if stamp != self.tf_stamp_s:
            self.tf_changed_wall_s = wall_s
            self.tf_stamp_s = stamp
        if wall_s - self.tf_changed_wall_s > self.max_feedback_gap_s:
            raise FeedbackUnavailable('Measured flange TF stopped updating in wall time.')
        p = transform.transform.translation
        return wall_s, self.clock_s, (p.x, p.y, p.z)


class PilotWindow:
    def __init__(self, run, settings, source, pyplot):
        from matplotlib.widgets import Button
        # Ubuntu 20.04's Matplotlib registers the 3D projection on import.
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        self.run, self.settings, self.source, self.plt = run, settings, source, pyplot
        self.index = 0
        self.trial = None
        self.ready_since = None
        self.latest_sample = None
        self.last_guard_wall = -math.inf
        self.closed = False
        self.figure = pyplot.figure(figsize=(9, 6), facecolor='#f5f7fa')
        self.figure.canvas.manager.set_window_title('MagPilot · virtual target-reaching pilot')
        self.figure.text(0.07, 0.92, 'Target-reaching pilot', size=23, weight='bold', color='#1d1d1f')
        self.caption = self.figure.text(0.07, 0.87, '', size=10, color='#677482')
        self.axes = self.figure.add_axes([0.07, 0.21, 0.55, 0.60], projection='3d')
        self.axes.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)',
                      xlim=(0.28, 0.62), ylim=(-0.18, 0.18), zlim=(0.23, 0.57))
        self.axes.set_title('Measured flange in {}'.format(run.manifest['robot_base_frame']), fontsize=10)
        self.axes.view_init(elev=22, azim=145)
        self.target_dot, = self.axes.plot([], [], [], 'o', color='#0a84ff', ms=11, label='Target')
        self.actual_dot, = self.axes.plot([], [], [], 'o', color='#34c759', ms=8, label='Actual flange')
        self.trail, = self.axes.plot([], [], [], color='#8abfff', lw=1.5)
        start = settings.start_position_m
        self.axes.plot([start[0]], [start[1]], [start[2]], '+', color='#778899', ms=12, label='Common start')
        self.axes.legend(fontsize=8, loc='upper left')
        self.detail = self.figure.text(0.65, 0.76, '', size=13, color='#1d1d1f', va='top', linespacing=1.7)
        self.status = self.figure.text(0.07, 0.14, 'Waiting for fresh simulated flange feedback…',
                                       size=11, color='#677482')
        self.figure.text(0.07, 0.055,
                         'Enter: start · Escape: cancel · control the robot in the Interface window',
                         size=9, color='#677482')
        self.button = Button(self.figure.add_axes([0.66, 0.24, 0.25, 0.075]),
                             'Start trial', color='#e5f1ff', hovercolor='#cce5ff')
        self.button.on_clicked(lambda _: self.start())
        self.figure.canvas.mpl_connect('key_press_event', self._key)
        self.figure.canvas.mpl_connect('close_event', self.close)
        self.timer = self.figure.canvas.new_timer(interval=50)
        self.timer.add_callback(self.tick)
        self.timer.start()
        self._show_target()

    def _participant_caption(self):
        metadata = self.run.manifest.get('metadata', {})
        name = metadata.get('participant_name', '').strip()
        label = '{} ({})'.format(name, self.run.participant_id) if name else self.run.participant_id
        experiment = metadata.get('experiment_name', '').strip()
        return '{} · {}'.format(experiment, label) if experiment else label

    def _show_target(self):
        targets = self.run.manifest['targets']
        if self.index == len(targets):
            self.target_dot.set_data_3d([], [], [])
            self.caption.set_text('{} · {} · finished'.format(self._participant_caption(), self.run.condition))
            self.detail.set_text('Run finished\n{} trial records\n\nsummary.csv\n+ trajectory CSVs'.format(len(self.run.completed)))
            self.status.set_text('Saved to {}'.format(self.run.output_dir))
            return
        target = targets[self.index]
        xyz = target['position_m']
        self.target_dot.set_data_3d([xyz[0]], [xyz[1]], [xyz[2]])
        self.caption.set_text('{} · condition {} · target {}/{}'.format(
            self._participant_caption(), self.run.condition, self.index + 1, len(targets)))
        self.detail.set_text('{} · repetition {}\n\nX {:.3f} m\nY {:.3f} m\nZ {:.3f} m\n\nTolerance {:.0f} mm\nHold {:.1f} s'.format(
            target['target_id'], target['repetition'], *xyz,
            self.settings.tolerance_m * 1000, self.settings.dwell_s))

    def _key(self, event):
        if event.key in ('enter', 'return'):
            self.start()
        elif event.key == 'escape':
            self.cancel()

    def start(self):
        if self.trial or self.index >= len(self.run.manifest['targets']):
            return
        try:
            self.source.verify_simulation()
            sample = self.source.sample()
            if (self.latest_sample is None
                    or sample[0] - self.latest_sample[0] > self.settings.max_feedback_gap_s):
                self.ready_since = None
            if self.ready_since is None or sample[0] - self.ready_since < self.settings.dwell_s:
                raise ValueError('Return to the common start and hold before starting.')
            self.trial = TargetTrial(self.run.manifest['targets'][self.index], self.settings,
                                     sample[0], sample[1], sample[2])
        except (FeedbackUnavailable, ValueError) as exc:
            self.status.set_text(str(exc))
            return
        self.ready_since = None
        self.trail.set_data_3d([], [], [])
        self.status.set_text('Recording — move the flange to the blue target and hold')

    def _save(self, result, advance):
        self.run.save_trial(self.trial)
        self.trial = None
        self.ready_since = None
        if advance:
            self.index += 1
        self._show_target()
        if self.index < len(self.run.manifest['targets']):
            self.status.set_text('{} — return to the common start; Enter starts the next trial'.format(result['status']))

    def cancel(self):
        if self.trial:
            now = time.monotonic()
            result = self.trial.finish('cancelled', now, self.trial.last_sim_s)
            self._save(result, advance=False)

    def tick(self):
        if self.closed:
            return
        try:
            if time.monotonic() - self.last_guard_wall >= 2:
                self.source.verify_simulation()
                self.last_guard_wall = time.monotonic()
            sample = self.source.sample()
            if (self.latest_sample is not None
                    and (sample[0] - self.latest_sample[0] > self.settings.max_feedback_gap_s
                         or sample[1] < self.latest_sample[1])):
                self.ready_since = None
            self.latest_sample = sample
        except FeedbackUnavailable as exc:
            self.ready_since = None
            if self.trial:
                result = self.trial.finish('feedback_lost', time.monotonic(), self.trial.last_sim_s)
                self._save(result, advance=False)
            self.status.set_text(str(exc))
            self.figure.canvas.draw_idle()
            return
        self.actual_dot.set_data_3d([sample[2][0]], [sample[2][1]], [sample[2][2]])
        if self.trial:
            result = self.trial.update(*sample)
            points = [(row['x_m'], row['y_m'], row['z_m']) for row in self.trial.rows]
            self.trail.set_data_3d(*zip(*points))
            if result:
                self._save(result, advance=result['status'] in ('completed', 'timed_out'))
            else:
                self.status.set_text('Recording {:.1f} s · target error {:.1f} mm'.format(
                    sample[0] - self.trial.start_wall_s,
                    distance(sample[2], self.trial.target['position_m']) * 1000))
        elif self.index < len(self.run.manifest['targets']):
            start_error = distance(sample[2], self.settings.start_position_m)
            if start_error > self.settings.start_tolerance_m:
                self.ready_since = None
                self.status.set_text('Return to common start ({:.0f} mm away)'.format(start_error * 1000))
            else:
                if self.ready_since is None:
                    self.ready_since = sample[0]
                if sample[0] - self.ready_since >= self.settings.dwell_s:
                    self.status.set_text('Ready at common start — Enter begins timing')
                else:
                    self.status.set_text('Hold the common starting position…')
        self.figure.canvas.draw_idle()

    def close(self, _event=None):
        if self.closed:
            return
        self.closed = True
        self.timer.stop()
        self.cancel()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', default=datetime.now().strftime('pilot_%Y%m%d_%H%M%S'))
    parser.add_argument('--output-dir', help='New directory; defaults to data_collection/virtual_task/<run-id>')
    parser.add_argument('--participant-id', default='pilot')
    parser.add_argument('--participant-name', default='')
    parser.add_argument('--experiment-name', default='')
    parser.add_argument('--condition', default='practice', help='Condition label only; does not change robot controls')
    parser.add_argument('--input-source', choices=('serial', 'trackpad'), default='trackpad',
                        help='Input used in the separate robot interface, saved as metadata')
    parser.add_argument('--magnet-count', type=int, choices=(1, 2, 3))
    parser.add_argument('--notes', default='')
    parser.add_argument('--arm-id', choices=('fr3', 'panda'), default='fr3')
    parser.add_argument('--tolerance-mm', type=float, default=20.0)
    parser.add_argument('--dwell-s', type=float, default=0.5)
    parser.add_argument('--timeout-s', type=float, default=60.0)
    parser.add_argument('--repetitions', type=int, default=1)
    parser.add_argument('--dry-run', action='store_true', help='Write a plan and empty summary; requires no ROS and records no trials')
    args = parser.parse_args(argv)
    try:
        run_id = safe_component(args.run_id)
        settings = TrialSettings(tolerance_m=args.tolerance_mm / 1000,
                                 dwell_s=args.dwell_s, timeout_s=args.timeout_s)
        targets = default_targets(args.repetitions)
        source = None if args.dry_run else RosFeedback(
            args.arm_id + '_link0', args.arm_id + '_link8', settings.max_feedback_gap_s)
        run = PilotRun(args.output_dir or os.path.join(_ROOT, 'data_collection', 'virtual_task', run_id),
                       run_id, args.participant_id, args.condition, settings, targets,
                       args.arm_id + '_link0', args.arm_id + '_link8',
                       metadata=dict(magnet_count=args.magnet_count, notes=args.notes,
                                     input_source=args.input_source,
                                     participant_name=args.participant_name,
                                     experiment_name=args.experiment_name,
                                     purpose='software_pilot', dry_run=args.dry_run))
        if args.dry_run:
            print('Plan only: {} targets; no measurements. {}'.format(len(targets), run.manifest_path))
            return 0
        import matplotlib
        matplotlib.use('TkAgg')
        matplotlib.rcParams['toolbar'] = 'None'
        import matplotlib.pyplot as plt
        window = PilotWindow(run, settings, source, plt)
        def stop(_signum, _frame):
            window.close()
            plt.close(window.figure)
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        try:
            plt.show()
        finally:
            window.close()
        print('Saved {} trial records to {}'.format(len(run.completed), run.output_dir))
        return 0
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print('Virtual task: {}'.format(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
