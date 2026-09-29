"""Tests of the bodywork mathematics against theory (no Blender needed).

.venv/bin/python experiments/006_benz_velo/test_bodywork.py
"""
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bodywork as bw  # noqa: E402

results = {}


def check(name, ok, detail):
    results[name] = (bool(ok), detail)
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")


# 1. Profiles: through the points, no overshoot on monotone data, C1 at the knots.
xs, ys = [0, 1, 2, 3, 4], [0, .1, 1, 1.05, 3]
p = bw.Profile(xs, ys)
dense = np.linspace(0, 4, 4001)
v = p(dense)
check('profile_interpolates', np.allclose(p(np.array(xs)), ys), 'passes through every point')
check('profile_no_overshoot', np.all(np.diff(v) >= -1e-12) and v.min() >= 0 and v.max() <= 3,
      f'monotone data gives a monotone curve within [{v.min():.3f}, {v.max():.3f}]')
eps = 1e-6
jumps = [abs((p(k+eps)-p(k))/eps-(p(k)-p(k-eps))/eps) for k in xs[1:-1]]
check('profile_c1', max(jumps) < 1e-3, f'largest slope jump at a knot {max(jumps):.2e}')

# 2. Sections: n = 2 is an ellipse; theta_at_height inverts the section.
th = np.linspace(0, 2*math.pi, 97)
yz = bw.superellipse(th, a=.7, up=.5, dn=.5, n_up=2, n_dn=2)
check('superellipse_n2_is_ellipse', np.allclose((yz[:, 0]/.7)**2+(yz[:, 1]/.5)**2, 1),
      'y^2/a^2 + z^2/b^2 = 1 at every sample')
err = []
for z in (-.3, 0, .2, .45):
    t = bw.theta_at_height(z, 0, .5, .4, 3, 2.5)
    err.append(abs(bw.superellipse([t], .7, .5, .4, 3, 2.5)[0, 1]-z))
check('theta_at_height_inverse', max(err) < 1e-9, f'height error {max(err):.1e} m')

# 3. Refinement meets the chord-height tolerance, with about the sagitta-law count.
R, tol = 1.0, .001
circle = lambda t: np.stack([R*np.cos(t), R*np.sin(t)], axis=-1)
ts = bw.refine(circle, [0, math.pi/2, math.pi, 1.5*math.pi, 2*math.pi], tol)
pts = circle(ts)
dev = max(R-np.linalg.norm((a+b)/2) for a, b in zip(pts, pts[1:]))
theory = 2*math.pi/(2*math.sqrt(2*R*tol))        # chord L = 2 sqrt(2 R s): N = 2 pi R / L
check('refine_meets_tolerance', dev <= tol, f'worst chord deviation {dev*1000:.3f} mm for tol {tol*1000:.1f} mm')
check('refine_count_near_theory', theory <= len(ts)-1 <= 2.2*theory,
      f'{len(ts)-1} segments, sagitta law gives at least {theory:.1f}')

# 4. Rotation-minimising frames on a helix: orthonormal, and no spin about the tangent.
s = np.linspace(0, 4*math.pi, 2000)
helix = np.stack([np.cos(s), np.sin(s), .3*s], axis=1)
t, r, b = bw.rmf(helix)
ortho = max(np.abs((t*r).sum(1)).max(), np.abs((t*b).sum(1)).max(), np.abs(np.linalg.norm(r, axis=1)-1).max())
spin = np.abs((np.diff(r, axis=0)*b[:-1]).sum(1)).max()    # dr . b = rotation about the tangent
check('rmf_orthonormal', ortho < 1e-6, f'largest dot/length error {ortho:.1e}')
check('rmf_no_twist', spin < 2e-5, f'largest spin increment {spin:.1e} rad per step')

# 5. Shell: a half-cylinder panel with a window is watertight, outward, with volume = area x t.
ang = np.linspace(0, math.pi, 25)
xs_ = np.linspace(0, 2, 21)
grid = np.stack([np.broadcast_to(xs_[:, None], (21, 25)), np.cos(ang)[None, :]*np.ones((21, 1)),
                 np.sin(ang)[None, :]*np.ones((21, 1))], axis=-1)
shell = bw.Shell(grid, .002, inward=lambda P: np.stack([np.zeros(P.shape[:2]), -P[..., 1], -P[..., 2]], axis=-1),
                 holes=[(5, 9, 4, 10)])
V, F = shell.mesh()
ok, info = bw.check_mesh(V, F)
kept = shell.keep.sum()
area = kept*(2/20)*(math.pi/24)           # quad area on the unit cylinder (to first order)
check('shell_watertight_outward', ok, f"{info['faces']} faces, open or mis-oriented edges {info['open_or_misoriented_edges']}")
check('shell_volume', abs(info['volume_m3']-area*.002)/(area*.002) < .02,
      f"volume {info['volume_m3']*1e6:.1f} cm3 against area x thickness {area*.002*1e6:.1f} cm3")

# 6. Patches: each within tol of a plane; they cover the panel (MuJoCo point-in-hull) and
#    leave the window open; the hull excess over the true surface is bounded by tol.
tol = .002
patches = shell.patches(tol)
planar = max(bw._planarity(pp[:len(pp)//2]) for pp in patches)
xml = ['<mujoco><asset>'] + [f'<mesh name="m{i}" vertex="{" ".join(f"{c:.6f}" for c in pp.reshape(-1))}"/>'
                             for i, pp in enumerate(patches)]
xml += ['</asset><worldbody>'] + [f'<geom name="g{i}" type="mesh" mesh="m{i}"/>' for i in range(len(patches))]
xml += ['<body name="probe" pos="0 0 0"><freejoint/><geom name="probe" type="sphere" size=".0005"/></body>',
        '</worldbody></mujoco>']
m = mujoco.MjModel.from_xml_string('\n'.join(xml))
d = mujoco.MjData(m)
probe = m.geom('probe').id
qadr = m.joint(0).qposadr[0]


def inside_any(pt):
    d.qpos[qadr:qadr+3] = pt
    d.qpos[qadr+3:qadr+7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    return min(mujoco.mj_geomDistance(m, d, probe, g, .05, None) for g in range(len(patches))) < 0


rng = np.random.default_rng(1)
covered, total, window_hits = 0, 0, 0
for _ in range(400):
    # A point in the built panel: bilinear within a quad, halfway through the thickness. (The hulls
    # contain the tessellated panel, not the ideal cylinder, which bulges past each chord.)
    i, j = rng.integers(0, 20), rng.integers(0, 24)
    u, w = rng.uniform(.02, .98, 2)
    mid = (shell.P+shell.Q)/2
    pt = ((1-u)*(1-w)*mid[i, j]+u*(1-w)*mid[i+1, j]+u*w*mid[i+1, j+1]+(1-u)*w*mid[i, j+1])
    if shell.keep[i, j]:
        total += 1
        covered += inside_any(pt)
    elif 5 < i < 8 and 4 < j < 9 and .2 < u < .8:   # well inside the window
        window_hits += inside_any(pt)
check('patches_planar', planar <= tol, f'{len(patches)} hulls, largest out-of-plane {planar*1000:.2f} mm (tol {tol*1000:.1f})')
check('patches_cover_panel', covered == total, f'{covered}/{total} random panel points inside a hull')
check('patches_leave_window_open', window_hits == 0, f'{window_hits} window points inside a hull')
excess = 0.0
for pp in patches:                     # hull vertices lie within [1 - t - tol, 1] of the axis
    rr = np.linalg.norm(pp[:, 1:], axis=1)
    excess = max(excess, (1-.002-rr).max())
check('hull_excess_bounded', excess <= tol+1e-9, f'hull points at most {excess*1000:.2f} mm inside the inner skin')

# 7. MuJoCo places mesh colliders where their vertices are, and measures distance between hulls.
xml2 = """<mujoco><asset><mesh name="a" vertex="1 1 1  1.1 1 1  1 1.1 1  1 1 1.1  1.1 1.1 1.1  1.1 1.1 1  1.1 1 1.1  1 1.1 1.1"/>
<mesh name="b" vertex="1.3 1 1  1.4 1 1  1.3 1.1 1  1.3 1 1.1  1.4 1.1 1.1  1.4 1.1 1  1.4 1 1.1  1.3 1.1 1.1"/></asset>
<worldbody><body name="k" pos=".5 0 0"><geom name="a" type="mesh" mesh="a"/><geom name="b" type="mesh" mesh="b"/></body></worldbody></mujoco>"""
m2 = mujoco.MjModel.from_xml_string(xml2)
d2 = mujoco.MjData(m2)
mujoco.mj_forward(m2, d2)
centre = d2.geom_xpos[m2.geom('a').id]
gap = mujoco.mj_geomDistance(m2, d2, 0, 1, .5, None)
check('mujoco_mesh_placement', np.allclose(centre, [1.55, 1.05, 1.05], atol=1e-6),
      f'hull centre at {np.round(centre, 4).tolist()} (vertices + body offset)')
check('mujoco_hull_distance', abs(gap-.2) < 1e-6, f'distance between cubes {gap:.6f} m (expected 0.2)')

failed = [k for k, (ok, _) in results.items() if not ok]
print(f'\n{len(results)-len(failed)}/{len(results)} passed')
sys.exit(1 if failed else 0)
