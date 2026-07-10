"""MagPilot brand assets: app icon (sleek paper-plane jet) + README banner.

Run inside the container:  python3 tools/make_logo.py
Writes docs/logo.png, docs/logo_small.png and docs/banner.png.
"""
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import FancyBboxPatch, Polygon, PathPatch
from matplotlib.path import Path
from matplotlib.transforms import Affine2D

_DOCS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs')

SKY_TOP, SKY_BOT = '#4aa3f5', '#0a5fd7'


def sky(ax, patch, x0, x1, y0, y1):
    """Vertical sky gradient clipped to a patch."""
    grad = np.linspace(0.0, 1.0, 256)[:, None] * np.ones((1, 2))
    im = ax.imshow(grad, extent=(x0, x1, y0, y1), origin='lower',
                   aspect='auto', zorder=0)
    im.set_cmap(LinearSegmentedColormap.from_list('sky', [SKY_BOT, SKY_TOP]))
    im.set_clip_path(patch)
    return im


def plane(ax, clip, cx, cy, s, angle, z=5):
    """Minimal paper-plane jet glyph, nose along +x, two-tone white."""
    tr = (Affine2D().scale(s).rotate_deg(angle).translate(cx, cy)
          + ax.transData)
    parts = [
        # upper wing
        Polygon([(1.00, 0.00), (-0.62, 0.72), (-0.34, 0.10)], closed=True,
                facecolor='#ffffff', edgecolor='none', zorder=z, transform=tr),
        # lower wing, slightly shaded for depth
        Polygon([(1.00, 0.00), (-0.34, -0.10), (-0.62, -0.72)], closed=True,
                facecolor='#cfe6fc', edgecolor='none', zorder=z, transform=tr),
        # fuselage fold
        Polygon([(1.00, 0.00), (-0.34, 0.10), (-0.50, 0.00), (-0.34, -0.10)],
                closed=True, facecolor='#eaf4ff', edgecolor='none',
                zorder=z + 1, transform=tr),
    ]
    for p in parts:
        p.set_clip_path(clip)
        ax.add_patch(p)


def contrail(ax, clip, pts, lw, z=4, alpha=0.85):
    codes = [Path.MOVETO] + [Path.CURVE4] * (len(pts) - 1)
    p = PathPatch(Path(pts, codes), facecolor='none', edgecolor='#ffffff',
                  lw=lw, alpha=alpha, capstyle='round', zorder=z)
    p.set_clip_path(clip)
    ax.add_patch(p)


# ── app icon ────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(5.12, 5.12), dpi=100)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.axis('off')
fig.patch.set_alpha(0.0)

icon = FancyBboxPatch((3, 3), 94, 94, boxstyle='round,pad=0,rounding_size=21',
                      facecolor='#0a5fd7', edgecolor='none', zorder=-1)
ax.add_patch(icon)
sky(ax, icon, 3, 97, 3, 97)

contrail(ax, icon, [(12, 16), (36, 10), (62, 22), (50, 44)], lw=5.0)
plane(ax, icon, 57, 57, 27, 36)

fig.savefig(os.path.join(_DOCS, 'logo.png'), transparent=True)
fig.savefig(os.path.join(_DOCS, 'logo_small.png'), dpi=9)  # ~46 px
plt.close(fig)

# ── README banner ───────────────────────────────────────────────────────────
figb = plt.figure(figsize=(16, 4.0), dpi=100)
axb = figb.add_axes([0, 0, 1, 1])
axb.set_xlim(0, 160); axb.set_ylim(0, 40); axb.axis('off')
figb.patch.set_alpha(0.0)

banner = FancyBboxPatch((1, 1), 158, 38,
                        boxstyle='round,pad=0,rounding_size=4.5',
                        mutation_aspect=0.25, facecolor='#0a5fd7',
                        edgecolor='none', zorder=-1)
axb.add_patch(banner)
sky(axb, banner, 1, 159, 1, 39)

contrail(axb, banner, [(8, 9), (20, 5), (34, 13), (29, 25)], lw=4.0)
plane(axb, banner, 34, 27, 9.0, 33)

axb.text(52, 24.0, 'MagPilot', fontsize=56, fontweight='bold',
         color='#ffffff', va='center', ha='left', zorder=6)
axb.text(52.4, 11.0, 'Pilot a robot arm with nothing but a magnet.',
         fontsize=19, color='#d7eaff', va='center', ha='left', zorder=6)

figb.savefig(os.path.join(_DOCS, 'banner.png'), transparent=True)
plt.close(figb)
print('logo + banner saved')
