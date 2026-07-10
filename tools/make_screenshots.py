"""Generate docs/ screenshots of the interface (Agg, headless)."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import os, sys
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DOCS = os.path.join(_ROOT, 'docs')
sys.path.insert(0, _ROOT)
import magnetometer_reader as mr

reader = mr.MagnetometerReader(
    input_source='touchpad', enable_classifier=False, ros=False,
    clean_view=True, writing_min_velocity=0.06,
    classifier_labels='ABCXLRUD0123',
)

# ---- synth a handwritten "A": two legs + crossbar (NaN gap = pen up) ----
e = reader.projection_extent
dt = 1.0 / 60.0

def line(p0, p1, v=0.085):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(2, int(np.linalg.norm(p1 - p0) / (v * dt)))
    ts = np.linspace(0.0, 1.0, n)[:, None]
    return p0 + ts * (p1 - p0)

apex = (0.05 * e, 0.62 * e)
seg = np.vstack([
    line((-0.42 * e, -0.55 * e), apex),          # left leg up
    line(apex, (0.52 * e, -0.55 * e)),           # right leg down
    np.full((6, 2), np.nan),                     # pen up
    line((-0.22 * e, -0.05 * e), (0.34 * e, -0.05 * e)),  # crossbar
])
px, py = seg[:, 0], seg[:, 1]
pz = np.full_like(px, 0.02)
t = np.arange(len(px)) * dt
mask, _, _ = reader.writing_sample_mask(px, py, pz, timestamps=t)
img = reader.pose_to_digit_image(px, py, pz, mask)
white = np.ones_like(img)

def fake_show(*a, **k):
    fig = plt.gcf()
    for art in fig.findobj():
        try:
            art.set_animated(False)
        except Exception:
            pass
    ax5 = next(ax for ax in fig.axes if ax.images)
    ax6 = next(ax for ax in fig.axes if len(getattr(ax, 'containers', [])) >= 2)
    ax5.images[0].set_data(img)
    fills = ax6.containers[1]
    demo = [0.97, 0.06, 0.03, 0.02] + [0.0] * (len(fills) - 4)
    for i, bar in enumerate(fills):
        bar.set_width(demo[i])
        bar.set_color('#0a84ff' if i == 0 else '#d1d1d6')
    ax6.set_yticklabels(list('ADXB0123CLRU')[:len(fills)], fontsize=10,
                        fontweight='bold', color='#1d1d1f')
    ax6.texts[0].set_text('Prediction: A (97.2%)\nRunner-up:  D (5.8%)\n'
                          'Ink: 141/240\nv: 0.084 m/s (ink 0.06-inf)\n'
                          'Z: 0.0200 [0.0200..0.0200]')
    fig.savefig(os.path.join(_DOCS, 'interface.png'), dpi=105,
                facecolor=fig.get_facecolor())
    print('saved interface')

    class _RobotMode:
        value = 'robot'
    reader.app_controller.mode = _RobotMode()
    reader._sync_teleop_ui()
    ax5.images[0].set_data(white)
    cursor = reader._teleop_ui.get('cursor')
    if cursor is not None:
        cursor.set_offsets(np.array([[0.15 * e, -0.1 * e]]))
        cursor.set_paths([reader._plane_path_for_heading(np.radians(28))])
    fig.savefig(os.path.join(_DOCS, 'magpilot.png'), dpi=105,
                facecolor=fig.get_facecolor())
    print('saved magpilot')

plt.show = fake_show
reader.is_running = True
reader.plot_data()
print('DONE')
