"""Procedural bodywork for the Traction Avant (run inside Blender by generate.py).

Design tables (x forward from the rear axle, metres) drive three shells built by
006_benz_velo/bodywork.py:
- cabin and boot: a loft of superellipse sections, with a rounded tail (the section shrinks to a
  point), side and rear windows and the rear wheel openings cut as (station, key) rectangles;
- bonnet: a loft of narrower sections from the bulkhead to the radiator;
- wings: per side, one sweep that arches over the front wheel, runs back as the running board
  and arches over the rear wheel.
All shapes are assumptions styled on the car's general form; none is sourced.
"""
import math

import numpy as np

TOL, SKIN = .002, .002      # chord-height tolerance and panel thickness (m)

# ---------- cabin and boot ----------
# Styled on photographs of 1934 7As (Wikimedia Commons, see the README): a short, steep, rounded
# back just behind the rear wings; a long flat roof with a slight visor; a high belt line
# continuing the bonnet top; shallow side glass.
CABIN = dict(
    top=([-.68, -.58, -.42, -.20, .10, .60, 1.20, 1.80, 1.975], [.98, 1.18, 1.34, 1.44, 1.49, 1.51, 1.51, 1.50, 1.46]),
    half_width=([-.68, -.55, -.35, .40, 1.20, 1.975], [.60, .68, .72, .735, .735, .72]),
    bottom=([-.68, -.55, -.47, 1.975], [.47, .44, .41, .41]),
    waist=.80, virtual_floor=0.0, n_up=4.0, n_dn=4.5, tumble=.20,
    belt=1.04, arch_top=.86, tail_dome=.07,
    stations=[-.75, -.68, -.47, -.36, -.18, .30, .47, 1.00, 1.10, 1.93, 1.975],
    windows=[(.30, 1.00), (1.10, 1.93)], rear_window=(-.36, -.18), wheel_opening=(-.47, .47))
BONNET = dict(x=(1.97, 3.47), top=([1.97, 3.0, 3.47], [1.04, 1.03, 1.00]),
              half_width=([1.97, 2.40, 3.0, 3.47], [.43, .42, .35, .25]),
              waist=.86, virtual_floor=.60, n_up=4.5, n_dn=4.0, bottom=.80)   # flatter top, defined shoulders


def cabin_shell(bw):
    c = CABIN
    top, width, bottom = (bw.Profile(*c[k]) for k in ('top', 'half_width', 'bottom'))
    x_end = c['stations'][1]                          # the tail dome runs from here to the first station

    def params(x):
        xx = max(x, x_end)
        t = float(top(xx))
        zc = min(c['waist'], t-.30)
        return dict(a=float(width(xx)), up=t-zc, dn=zc-c['virtual_floor'], n_up=c['n_up'], n_dn=c['n_dn'],
                    zc=zc, tumble=c['tumble'], top=t)

    def scale(x):   # quarter-ellipse dome: the tail section shrinks to 8% at the very end
        if x >= x_end:
            return 1.0
        u = (x_end-x)/c['tail_dome']
        return max(.08, math.sqrt(max(0.0, 1-u*u)))

    def section(x, theta):
        p = params(x)
        yz = bw.superellipse(theta, p['a'], p['up'], p['dn'], p['n_up'], p['n_dn'], p['zc'], p['tumble'])
        k = scale(x)
        centre = np.array([0.0, (p['top']+float(bottom(max(x, x_end))))/2])
        return centre+(yz-centre)*k

    def keys(x):
        p = params(x)
        z = lambda h: bw.theta_at_height(h, p['zc'], p['up'], p['dn'], p['n_up'], p['n_dn'])
        zb = float(bottom(max(x, x_end)))
        belt = min(c['belt'], p['top']-.06)
        cant = min(1.42, p['top']-.03)
        arch = min(c['arch_top'], belt-.03)
        right = [z(zb), z(arch), z(belt), z(cant)]
        for i in range(1, 4):                          # keep strictly increasing
            right[i] = max(right[i], right[i-1]+1e-3)
        return right+[math.pi-t for t in reversed(right)]

    loft = bw.Loft(section, keys, c['stations'], TOL)
    holes = [loft.hole(*c['wheel_opening'], 0, 1), loft.hole(*c['wheel_opening'], 6, 7),
             loft.hole(*c['rear_window'], 3, 4)]
    for x0, x1 in c['windows']:
        holes += [loft.hole(x0, x1, 2, 3), loft.hole(x0, x1, 4, 5)]
    centre = lambda P: np.stack([np.zeros(P.shape[:2]), -P[..., 1], .9-P[..., 2]], axis=-1)
    return bw.Shell(loft.P, SKIN, inward=centre, holes=holes), loft


def bonnet_shell(bw):
    b = BONNET
    top, width = bw.Profile(*b['top']), bw.Profile(*b['half_width'])

    def params(x):
        t = float(top(x))
        return dict(a=float(width(x)), up=t-b['waist'], dn=b['waist']-b['virtual_floor'], n_up=b['n_up'],
                    n_dn=b['n_dn'], zc=b['waist'])

    def section(x, theta):
        p = params(x)
        return bw.superellipse(theta, p['a'], p['up'], p['dn'], p['n_up'], p['n_dn'], p['zc'])

    def keys(x):
        p = params(x)
        t = bw.theta_at_height(b['bottom'], p['zc'], p['up'], p['dn'], p['n_up'], p['n_dn'])
        return [t, math.pi/2, math.pi-t]

    loft = bw.Loft(section, keys, list(b['x']), TOL)
    centre = lambda P: np.stack([np.zeros(P.shape[:2]), -P[..., 1], .75-P[..., 2]], axis=-1)
    return bw.Shell(loft.P, SKIN, inward=centre), loft


# ---------- grille ----------
GRILLE = dict(half_width=.25, bottom=.42, shoulder=.90, crown=.14, n=2.5, rake=.10, x_bottom=3.53, depth=.02)


def grille_outline(u):
    """Front-view outline height at u in [-1, 1] across: straight sides to the shoulder, then a
    superellipse arch to the crown."""
    g = GRILLE
    return g['shoulder']+g['crown']*(1-np.abs(u)**g['n'])**(1/g['n'])


def grille_x(z):
    """The shield leans back: x falls by `rake` from the bottom to the top."""
    g = GRILLE
    return g['x_bottom']-g['rake']*(z-g['bottom'])/(g['shoulder']+g['crown']-g['bottom'])


def grille_shell(bw):
    g = GRILLE
    u = np.linspace(-1, 1, 41)
    t = np.linspace(0, 1, 13)
    z = g['bottom']+t[None, :]*(grille_outline(u)[:, None]-g['bottom'])
    grid = np.stack([grille_x(z), np.broadcast_to(g['half_width']*u[:, None], z.shape), z], axis=-1)
    back = lambda P: np.stack([-np.ones(P.shape[:2]), np.zeros(P.shape[:2]), np.zeros(P.shape[:2])], axis=-1)
    return bw.Shell(grid, g['depth'], inward=back)


def grille_surround():
    """Centreline of the chrome surround along the outline (open at the bottom)."""
    g = GRILLE
    u = np.linspace(-1, 1, 61)
    z = grille_outline(u)
    top = [(float(grille_x(zz)), float(g['half_width']*uu), float(zz)) for uu, zz in zip(u, z)]
    side = lambda y: [(float(grille_x(zz)), y, float(zz)) for zz in np.linspace(g['bottom'], g['shoulder'], 8)]
    return side(-g['half_width'])[:-1]+top+list(reversed(side(g['half_width'])))[1:]


# ---------- wings and running boards ----------
WING = dict(front_centre=(2.91, .34), front_radius=.55, front_arc_deg=(12, 155),
            rear_centre=(0.0, .34), rear_radius=.52, rear_arc_deg=(25, 150),   # the flare covers the body's wheel opening
            board=(2.15, .70, .40), tail=(-.62, .47),
            # Rear: a flare outboard of the body side (the rear track is narrower than the body).
            y=([-.62, .43, .70, 2.15, 2.44, 3.40], [.83, .83, .80, .80, .67, .67]),
            # Outboard half deep (the skirt comes well down over the tyre's side), inboard half
            # shallow and wide (towards the bonnet) but kept clear of the steered tyre.
            half_out=([-.62, .43, .70, 2.15, 2.44, 3.46], [.10, .10, .09, .09, .15, .15]),
            # Inboard, the front wing reaches y = 0.44 m, 25 mm outboard of the longerons (it meets
            # the bonnet side on the car); its edge stays above the steered tyre's reach.
            half_in=([-.62, .43, .70, 2.15, 2.44, 3.46], [.09, .09, .09, .09, .23, .23]),
            depth_out=([-.62, .43, .70, 2.15, 2.44, 3.46], [.12, .12, .02, .02, .16, .16]),
            depth_in=([-.62, .43, .70, 2.15, 2.44, 3.46], [.04, .04, .02, .02, .04, .04]),
            n=2.6)


def wing_path(bw):
    """Dense x-z path, front to rear: arc over the front wheel, a cubic Bezier down to the board
    (tangent-continuous at both ends), the board, a Bezier up to the rear arc, the rear arc, and
    a Bezier down to the tail."""
    w = WING
    arc = lambda c, R, a0, a1: np.array([[c[0]+R*math.cos(a), c[1]+R*math.sin(a)]
                                         for a in np.radians(np.linspace(a0, a1, 90))])
    tangent = lambda a: np.array([-math.sin(a), math.cos(a)])   # direction of increasing angle
    fa, ra = np.radians(w['front_arc_deg']), np.radians(w['rear_arc_deg'])
    front = arc(w['front_centre'], w['front_radius'], *w['front_arc_deg'])
    rear = arc(w['rear_centre'], w['rear_radius'], *w['rear_arc_deg'])
    x0, x1, zb = w['board']
    down = bw.bezier([front[-1], front[-1]+.14*tangent(fa[1]), [x0+.14, zb], [x0, zb]], np.linspace(0, 1, 40))
    board = np.array([[x, zb] for x in np.linspace(x0, x1, 60)])
    up = bw.bezier([[x1, zb], [x1-.12, zb], rear[0]-.12*tangent(ra[0]), rear[0]], np.linspace(0, 1, 40))
    tail = bw.bezier([rear[-1], rear[-1]+.10*tangent(ra[1]), [w['tail'][0]+.08, w['tail'][1]], list(w['tail'])],
                     np.linspace(0, 1, 30))
    pts = np.concatenate([front, down[1:], board[1:], up[1:], rear[1:], tail[1:]])
    return pts


def wing_shell(bw, side):
    w = WING
    xz = wing_path(bw)
    y = bw.Profile(*w['y'])
    ho, hi, do, di = (bw.Profile(*w[k]) for k in ('half_out', 'half_in', 'depth_out', 'depth_in'))
    path = np.column_stack([xz[:, 0], side*y(xz[:, 0]), xz[:, 1]])
    phi = np.linspace(0, math.pi, 41)
    # The frames' lateral axis points to -y along this path (it runs rearward), so positive
    # lateral is outboard for the right wing and inboard for the left.
    outer = np.sign(np.cos(phi)) == -side

    def section(s, i):
        x = xz[i, 0]
        a = np.where(outer, float(ho(x)), float(hi(x)))
        h = np.where(outer, float(do(x)), float(di(x)))
        n = w['n']
        lat = a*np.sign(np.cos(phi))*np.abs(np.cos(phi))**(2/n)
        nor = h*np.abs(np.sin(phi))**(2/n)-h          # crown on the path, skirts down by h
        return np.column_stack([lat, nor])

    grid, _ = bw.sweep(path, section, TOL)
    below = lambda P: np.stack([np.zeros(P.shape[:2]), np.zeros(P.shape[:2]), -np.ones(P.shape[:2])], axis=-1)
    # Inward is toward the wheel: away from the crown's outward normal, i.e. downward/inward.
    return bw.Shell(grid, SKIN, inward=lambda P: _toward_axle(P, w)), path


def _toward_axle(P, w):
    """Direction from a wing point toward the nearer wheel centre (or straight down on the board)."""
    out = np.zeros_like(P)
    for c in (w['front_centre'], w['rear_centre']):
        d = np.stack([c[0]-P[..., 0], np.zeros(P.shape[:2]), c[1]-P[..., 2]], axis=-1)
        near = np.abs(P[..., 0]-c[0]) < .6
        out[near] = d[near]
    board = ~((np.abs(P[..., 0]-w['front_centre'][0]) < .6) | (np.abs(P[..., 0]-w['rear_centre'][0]) < .6))
    out[board] = [0, 0, -1]
    return out


def build(bw, m, V, part):
    """Make the shells, check each is watertight, and add them as parts with their patches."""
    report = {}
    cabin, loft = cabin_shell(bw)
    pieces = [('Body shell', cabin, m['paint'], dict(joins=['Frame rail 1', 'Frame rail -1', 'Front bulkhead']))]
    bonnet, _ = bonnet_shell(bw)
    pieces.append(('Bonnet', bonnet, m['paint'], dict(joins=['Front bulkhead', 'Scuttle', 'Grille'])))
    pieces.append(('Grille', grille_shell(bw), m['iron'], dict(joins=['Radiator'])))
    for s in (1, -1):
        wing, _ = wing_shell(bw, s)
        pieces.append((f'Wing {s}', wing, m['wing'], dict(joins=[f'Frame rail {s}'])))
    for name, shell, mat, links in pieces:
        Vtx, F = shell.mesh()
        ok, info = bw.check_mesh(Vtx, F)
        patches = shell.patches(TOL)
        obj = bw.to_blender(name, Vtx, F, mat, V, patches=patches)
        part(obj, **links)
        report[name] = dict(watertight_outward=bool(ok), open_or_misoriented_edges=int(info['open_or_misoriented_edges']),
                            volume_m3=float(info['volume_m3']), faces=int(info['faces']), patches=len(patches),
                            grid=[int(v) for v in shell.P.shape[:2]])
        if not ok:
            raise RuntimeError(f'{name} is not a closed, outward shell: {info}')
    report['tolerance_m'], report['thickness_m'] = TOL, SKIN
    report['cabin_stations'] = int(len(loft.x))
    return report
