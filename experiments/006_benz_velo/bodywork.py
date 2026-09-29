"""Procedural bodywork from parametric curves (pure numpy; the Blender side is to_blender()).

Principles
- Profiles are shape-preserving cubic Hermite functions (Fritsch-Carlson): C1, and a body line
  never overshoots the designer's points (a natural cubic spline does).
- Sections are superellipses |y/a|^n + |z/b|^n = 1 with separate upper and lower halves and a
  tumblehome term: n = 2 is an ellipse, n -> infinity a rectangle; 2.5-4 reads as coachwork.
- Surfaces are lofts (sections along x) and sweeps (a section along a path, carried by
  rotation-minimising frames so the section does not twist).
- Tessellation is set by a chord-height tolerance: a chord of length L on curvature k deviates by
  the sagitta L^2 k / 8; spacing is refined until every deviation is below `tol`.
- Panels are shells: the surface offset inward by the thickness, closed at every boundary.
- Collision: each block of the built (tessellated) shell is collided as its convex hull. The hull
  contains the block and exceeds it by at most the block's out-of-plane spread (kept below `tol`),
  so clearances are measured on the pessimistic side by at most `tol`. The tessellation itself
  lies within `tol` of the design surface; the ideal surface bulges past each chord by up to
  that much, which is why the part is the mesh, not the formula.
"""
import math

import numpy as np


# ---------- curves ----------

class Profile:
    """Monotone (shape-preserving) cubic Hermite interpolant through (x, y) points."""

    def __init__(self, xs, ys):
        self.x = np.asarray(xs, dtype=float)
        self.y = np.asarray(ys, dtype=float)
        if np.any(np.diff(self.x) <= 0):
            raise ValueError('profile x must increase')
        h = np.diff(self.x)
        d = np.diff(self.y)/h
        m = np.zeros_like(self.y)
        m[0], m[-1] = d[0], d[-1]
        for i in range(1, len(self.x)-1):
            if d[i-1]*d[i] <= 0:
                m[i] = 0.0                     # a turning point stays at the designer's point
            else:
                w1, w2 = 2*h[i]+h[i-1], h[i]+2*h[i-1]
                m[i] = (w1+w2)/(w1/d[i-1]+w2/d[i])   # weighted harmonic mean (Fritsch-Butland)
        self.m = m

    def __call__(self, x):
        x = np.clip(np.asarray(x, dtype=float), self.x[0], self.x[-1])
        i = np.clip(np.searchsorted(self.x, x)-1, 0, len(self.x)-2)
        h = self.x[i+1]-self.x[i]
        t = (x-self.x[i])/h
        t2, t3 = t*t, t*t*t
        return ((2*t3-3*t2+1)*self.y[i]+(t3-2*t2+t)*h*self.m[i]
                + (-2*t3+3*t2)*self.y[i+1]+(t3-t2)*h*self.m[i+1])


def bezier(ctrl, t):
    """Point(s) on a Bezier curve of any degree (de Casteljau)."""
    p = np.asarray(ctrl, dtype=float)
    t = np.atleast_1d(np.asarray(t, dtype=float))[:, None, None]
    pts = np.broadcast_to(p, (t.shape[0],)+p.shape).copy()
    while pts.shape[1] > 1:
        pts = (1-t)*pts[:, :-1]+t*pts[:, 1:]
    return pts[:, 0]


def _to_chord(p, a, b):
    """Distance from p to the line through a and b (any dimension)."""
    ab, v = b-a, p-a
    L = np.linalg.norm(ab)
    if L == 0:
        return float(np.linalg.norm(v))
    u = ab/L
    return float(np.linalg.norm(v-(v@u)*u))


def sagitta(points):
    """Largest distance from each interior point to the chord of its neighbours."""
    p = np.asarray(points, dtype=float)
    if len(p) < 3:
        return 0.0
    a, b, c = p[:-2], p[1:-1], p[2:]
    ac = c-a
    n = np.linalg.norm(ac, axis=1, keepdims=True)
    u = np.divide(ac, n, out=np.zeros_like(ac), where=n > 0)
    v = b-a
    return float(np.max(np.linalg.norm(v-(v*u).sum(axis=1, keepdims=True)*u, axis=1)))


def refine(f, keys, tol, max_depth=12):
    """Parameters between (and including) the key values such that the polyline through f(t)
    deviates from the curve by less than tol: each interval is split until its midpoint lies
    within tol of the chord. f maps an array of t to points (n, d)."""
    out = [keys[0]]
    for a, b in zip(keys, keys[1:]):
        stack = [(a, b, 0)]
        pieces = []
        while stack:
            lo, hi, depth = stack.pop()
            mid = (lo+hi)/2
            pa, pm, pb = f(np.array([lo, mid, hi]))
            dev = _to_chord(pm, pa, pb)
            # also test quarter points, so an S-bend between endpoints cannot hide
            q = f(np.array([(3*lo+hi)/4, (lo+3*hi)/4]))
            dq = max(np.linalg.norm(q[0]-(pa+pm)/2), np.linalg.norm(q[1]-(pm+pb)/2))
            if (dev > tol or dq > 4*tol) and depth < max_depth:
                stack += [(mid, hi, depth+1), (lo, mid, depth+1)]
            else:
                pieces.append(hi)
        out += sorted(pieces)
    return np.array(out)


# ---------- sections ----------

def superellipse(theta, a, up, dn, n_up, n_dn, zc=0.0, tumble=0.0):
    """Points (y, z) of an asymmetric superellipse; theta = 0 is the right waist (-y), pi/2 the
    top, pi the left waist. tumble narrows the upper half: y *= 1 - tumble*((z-zc)/up)^2."""
    theta = np.asarray(theta, dtype=float)
    c, s = np.cos(theta), np.sin(theta)
    n = np.where(s >= 0, n_up, n_dn)
    b = np.where(s >= 0, up, dn)
    y = -a*np.sign(c)*np.abs(c)**(2/n)
    z = zc+b*np.sign(s)*np.abs(s)**(2/n)
    rise = np.clip((z-zc)/max(up, 1e-9), 0, 1)
    return np.stack([y*(1-tumble*rise**2), z], axis=-1)


def theta_at_height(z, zc, up, dn, n_up, n_dn):
    """Angle in [-pi/2, pi/2] on the right half where the section reaches height z."""
    if z >= zc:
        s = min(1.0, ((z-zc)/up)**(n_up/2))
        return math.asin(s)
    s = min(1.0, ((zc-z)/dn)**(n_dn/2))
    return -math.asin(s)


# ---------- rotation-minimising frames (double reflection, Wang et al. 2008) ----------

def rmf(points, up=(0, 0, 1)):
    """Frames (tangent, normal, binormal) along a polyline without twist."""
    p = np.asarray(points, dtype=float)
    t = np.gradient(p, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    r = np.cross(up, t[0])
    if np.linalg.norm(r) < 1e-9:
        r = np.cross([1, 0, 0], t[0])
    r /= np.linalg.norm(r)
    frames = [r]
    for i in range(len(p)-1):
        v1 = p[i+1]-p[i]
        c1 = v1@v1
        rl = frames[-1]-(2/c1)*(v1@frames[-1])*v1
        tl = t[i]-(2/c1)*(v1@t[i])*v1
        v2 = t[i+1]-tl
        c2 = v2@v2
        frames.append(rl-(2/c2)*(v2@rl)*v2 if c2 > 1e-18 else rl)
    r = np.array(frames)
    return t, r, np.cross(t, r)


# ---------- lofts ----------

def _arc_resample(pts, n):
    """n+1 points evenly spaced by arc length along a dense polyline."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seg)])
    target = np.linspace(0, cum[-1], n+1)
    return np.stack([np.interp(target, cum, pts[:, k]) for k in range(pts.shape[1])], axis=1)


def _needed(pts, tol):
    """Segments needed for a dense polyline by the sagitta law: L = sqrt(8 tol / k)."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    length = seg.sum()
    if length == 0:
        return 1
    a, b, c = pts[:-2], pts[1:-1], pts[2:]
    ab, bc, ac = (np.linalg.norm(b-a, axis=1), np.linalg.norm(c-b, axis=1), np.linalg.norm(c-a, axis=1))
    cross = np.abs((b-a)[:, 0]*(c-a)[:, 1]-(b-a)[:, 1]*(c-a)[:, 0])
    k = np.max(2*cross/np.maximum(ab*bc*ac, 1e-15)) if len(a) else 0.0   # Menger curvature
    return max(1, int(math.ceil(length*math.sqrt(max(k, 1e-9)/(8*tol)))))


class Loft:
    """A surface lofted from 2-D sections (y, z) along x.

    section(x, theta) -> (n, 2) points; keys(x) -> increasing thetas, the same number at every x,
    that split each section into intervals (sill edge, belt line, cant rail...). Holes are then
    rectangles in (station, key) space. Stations: the given key x values, refined until each
    section lies within tol of the average of its neighbours; points per interval from the
    sagitta law at the most curved station."""

    def __init__(self, section, keys, x_keys, tol, dense=400):
        self.section, self.keys, self.tol, self.dense = section, keys, tol, dense
        probe = np.linspace(x_keys[0], x_keys[-1], 25)
        nk = len(keys(x_keys[0]))
        self.counts = [max(_needed(self._dense(x, k), tol) for x in probe) for k in range(nk-1)]
        xs = [x_keys[0]]
        for a, b in zip(x_keys, x_keys[1:]):
            xs += list(self._refine_x(a, b, self.row(a), self.row(b), 0))+[b]
        self.x = np.array(xs)
        self.P = np.array([self.row(x) for x in self.x])
        self.key_columns = np.concatenate([[0], np.cumsum(self.counts)])

    def _dense(self, x, k):
        t = self.keys(x)
        return self.section(x, np.linspace(t[k], t[k+1], self.dense))

    def row(self, x):
        pieces = []
        for k, n in enumerate(self.counts):
            pts = _arc_resample(self._dense(x, k), n)
            pieces.append(pts if k == 0 else pts[1:])
        yz = np.concatenate(pieces)
        return np.column_stack([np.full(len(yz), x), yz])

    def _refine_x(self, a, b, ra, rb, depth):
        m = (a+b)/2
        rm = self.row(m)
        if depth >= 10 or np.max(np.linalg.norm(rm-(ra+rb)/2, axis=1)) <= self.tol:
            return []
        return self._refine_x(a, m, ra, rm, depth+1)+[m]+self._refine_x(m, b, rm, rb, depth+1)

    def station(self, x):
        """Index of the station nearest x (key x values are stations exactly)."""
        return int(np.argmin(np.abs(self.x-x)))

    def hole(self, x0, x1, k0, k1):
        """Hole spec for Shell: quads between stations x0..x1 and keys k0..k1."""
        return (self.station(x0), self.station(x1), int(self.key_columns[k0]), int(self.key_columns[k1]))


def sweep(path, section, tol, up=(0, 0, 1)):
    """Grid of a section carried along a path on rotation-minimising frames.
    path: (n, 3) dense points; section(s, i) -> (m, 2) points (lateral, normal) for arc-length
    fraction s at path index i, the same m everywhere. Rows are resampled so the surface lies
    within tol of the dense sweep along the path."""
    path = np.asarray(path, dtype=float)
    t, r, b = rmf(path, up)
    rows = []
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    s /= s[-1]
    for i in range(len(path)):
        sec = np.asarray(section(s[i], i), dtype=float)
        rows.append(path[i]+sec[:, :1]*r[i]+sec[:, 1:2]*b[i])
    rows = np.array(rows)
    keep = [0]
    for i in range(1, len(rows)-1):    # drop rows the neighbours reproduce within tol
        if np.max(np.linalg.norm(rows[i]-(rows[keep[-1]]+rows[i+1])/2, axis=1)) > tol/2:
            keep.append(i)
    keep.append(len(rows)-1)
    return rows[keep], s[keep]


# ---------- shells ----------

class Shell:
    """A thin panel: outer grid P[i, j] (i along the surface, j across), thickness inward,
    optional holes (blocks of quads to leave out). Builds a watertight mesh and convex patches."""

    def __init__(self, grid, thickness, inward, holes=None):
        self.P = np.asarray(grid, dtype=float)
        self.t = thickness
        ni, nj = self.P.shape[:2]
        self.keep = np.ones((ni-1, nj-1), dtype=bool)
        for (i0, i1, j0, j1) in holes or []:
            self.keep[i0:i1, j0:j1] = False
        # Vertex normals from the grid tangents, flipped to point away from `inward`.
        du = np.gradient(self.P, axis=0)
        dv = np.gradient(self.P, axis=1)
        n = np.cross(du, dv)
        n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-12)
        toward = np.asarray(inward(self.P)) if callable(inward) else np.asarray(inward)-self.P
        flip = (n*toward).sum(axis=2) > 0
        n[flip] *= -1
        self.N = n
        # Winding: quads (i,j)->(i+1,j)->(i+1,j+1) face along du x dv; reverse if that points inward.
        raw = np.cross(du, dv)
        self.reverse = float((raw*n).sum()) < 0
        self.Q = self.P-thickness*n                  # inner skin

    def mesh(self):
        """Vertices and quad faces (outer, inner reversed, and rims on every boundary edge)."""
        ni, nj = self.P.shape[:2]
        idx = lambda i, j, inner: (ni*nj if inner else 0)+i*nj+j
        V = np.concatenate([self.P.reshape(-1, 3), self.Q.reshape(-1, 3)])
        F = []
        for i in range(ni-1):
            for j in range(nj-1):
                if self.keep[i, j]:
                    F.append((idx(i, j, 0), idx(i+1, j, 0), idx(i+1, j+1, 0), idx(i, j+1, 0)))
                    F.append((idx(i, j, 1), idx(i, j+1, 1), idx(i+1, j+1, 1), idx(i+1, j, 1)))
        # Boundary edges of the kept region: an edge used by exactly one kept quad.
        use = {}
        for i in range(ni-1):
            for j in range(nj-1):
                if not self.keep[i, j]:
                    continue
                for e in (((i, j), (i+1, j)), ((i+1, j), (i+1, j+1)), ((i+1, j+1), (i, j+1)), ((i, j+1), (i, j))):
                    key = tuple(sorted(e))
                    use.setdefault(key, []).append(e)
        for key, es in use.items():
            if len(es) == 1:
                (a, b) = es[0]
                F.append((idx(*b, 0), idx(*a, 0), idx(*a, 1), idx(*b, 1)))
        if self.reverse:
            F = [f[::-1] for f in F]
        return V, F

    def patches(self, tol, max_block=8):
        """Convex pieces covering the kept shell. Blocks of quads grow while they stay within tol
        of a plane (so each hull exceeds its block by at most tol) and are entirely panel or
        entirely opening; only panel blocks are returned."""
        ni, nj = self.P.shape[:2]
        out = []
        i = 0
        while i < ni-1:
            bi = 1
            while (bi < max_block and i+bi < ni-1 and (self.keep[i+bi] == self.keep[i]).all()
                   and self._flat(i, i+bi+2, 0, nj, tol, rows_only=True)):   # with the row it would add
                bi += 1
            j = 0
            while j < nj-1:
                bj = 1
                kept = self.keep[i, j]
                while (bj < max_block and j+bj < nj-1 and (self.keep[i:i+bi, j+bj] == kept).all()
                       and self._flat(i, i+bi+1, j, j+bj+2, tol)):   # with the column it would add
                    bj += 1
                if kept:
                    out.append(np.concatenate([self.P[i:i+bi+1, j:j+bj+1].reshape(-1, 3),
                                               self.Q[i:i+bi+1, j:j+bj+1].reshape(-1, 3)]))
                j += bj
            i += bi
        return out

    def _flat(self, i0, i1, j0, j1, tol, rows_only=False):
        pts = self.P[i0:i1, j0:j1].reshape(-1, 3)
        if rows_only:   # a strip is judged row by row across (the full width is never planar)
            return all(_planarity(self.P[i0:i1, j:j+2].reshape(-1, 3)) <= tol for j in range(j0, j1-1))
        return _planarity(pts) <= tol


def _planarity(pts):
    c = pts.mean(axis=0)
    _, s, vt = np.linalg.svd(pts-c, full_matrices=False)
    return float(np.max(np.abs((pts-c)@vt[-1])))


def check_mesh(V, F):
    """Watertightness and orientation: every edge in exactly two faces with opposite directions,
    and a positive enclosed volume (outward normals). Returns (ok, details)."""
    edges = {}
    for f in F:
        for a, b in zip(f, f[1:]+f[:1]):
            edges.setdefault((a, b), 0)
            edges[(a, b)] += 1
    bad = [e for e, c in edges.items() if c != 1 or edges.get((e[1], e[0]), 0) != 1]
    vol = 0.0
    for f in F:
        a = V[f[0]]
        for b, c in zip(f[1:-1], f[2:]):
            vol += np.dot(a, np.cross(V[b], V[c]))/6
    return (not bad and vol > 0), dict(open_or_misoriented_edges=len(bad), volume_m3=vol, faces=len(F))


# ---------- Blender ----------

def to_blender(name, V, F, mat, collection, patches=None, smooth=True):
    """Mesh object from vertices and faces; stores the convex patches for the exporter."""
    import bpy
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in V], [], [tuple(f) for f in F])
    me.validate()
    if smooth:
        for poly in me.polygons:
            poly.use_smooth = True
    obj = bpy.data.objects.new(name, me)
    collection.objects.link(obj)
    obj.data.materials.append(mat)
    if patches is not None:
        obj['patches'] = [list(np.asarray(p, dtype=float).reshape(-1)) for p in patches]
    return obj
