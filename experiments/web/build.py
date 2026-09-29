"""Build the multi-car web simulator in experiments/web/output/site (git-ignored).

.venv/bin/python experiments/web/build.py [--page-only] [--serve [PORT]]   (default port 8321; open /)

For every car in cars.py:
- model_flat.xml / model_bumps.xml: the car's validated full model exactly as its last run.py wrote
  it (output/<stage>/physics/velo.xml: engine, springs), with mesh paths reduced to file names; the
  bump version adds the bars of 010's bumpy_road.py course as road geoms, nothing else changed
- car.glb: the car's appearance from its saved .blend (export_glb.py), each part in its body frame
- config: engine, gears, brake devices and capacities, steering, suspension, and a Python parity
  reference: the car's own control.py driving the same model (launch in first gear, pedal brake at
  4 s), which parity_test.mjs must reproduce
Then mujoco.js/.wasm (the pinned version matching Python's MuJoCo), manifest.json and the page.
"""
import hashlib
import importlib.util
import json
import re
import shutil
import subprocess
import sys
import textwrap
import urllib.request
from pathlib import Path

import mujoco

WEB = Path(__file__).resolve().parent
EXP = WEB.parent
REPO = EXP.parent
SITE = WEB/'output/site'
sys.path.insert(0, str(WEB))
from cars import CARS  # noqa: E402

PARITY = """
import json, sys, mujoco
sys.path.insert(0, {exp!r})
import control as C
m = mujoco.MjModel.from_xml_path({xml!r})
d = mujoco.MjData(m)
ctrl = C.Controller(gears=[(.5, {first!r})], brake=C.Brake(apply_at=4.0, **{brake!r}))
rows = []
while d.time < 8.0 - 1e-9:
    ctrl(m, d)
    mujoco.mj_step(m, d)
    if abs(d.time/.5 - round(d.time/.5)) < 1e-6:
        rows.append([round(d.time, 3), float(d.qpos[0]), float(d.qpos[1]), float((d.qvel[0]**2+d.qvel[1]**2)**.5)])
print(json.dumps(rows))
"""


def course():
    spec = importlib.util.spec_from_file_location('bumpy', EXP/'010_ford_model_t_1909/bumpy_road.py')
    bumpy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bumpy)
    span = dict(full=(-1.3, 1.3), left=(.25, 1.3), right=(-1.3, -.25))
    lines = [f'    <geom name="road_bar_{i}" type="capsule" size="{h}" fromto="{x} {span[s][0]} 0 {x} {span[s][1]} 0" '
             f'contype="1" conaffinity="6" rgba=".5 .45 .35 1" />' for i, (x, h, s) in enumerate(bumpy.BARS)]
    return lines, [dict(x=x, height=h, side=s) for x, h, s in bumpy.BARS]


def build_car(car, bar_lines):
    exp = EXP/car['exp']
    stage = car.get('stage', 'full')
    physics = exp/'output'/stage/'physics'
    out = SITE/'cars'/car['id']
    out.mkdir(parents=True, exist_ok=True)
    geometry = json.loads((physics/'geometry.json').read_text())
    report = json.loads((physics/'report.json').read_text())
    if not report.get('passed'):
        raise SystemExit(f"{car['id']}: its last validation did not pass; not building it")
    xml = (physics/'velo.xml').read_text()
    meshes = re.findall(r'file="([^"]+\.obj)"', xml)
    xml = re.sub(r'file="[^"]*/([^"/]+\.obj)"', r'file="\1"', xml)
    (out/'model_flat.xml').write_text(xml)
    slope = re.search(r'\n(\s*<geom name="slope"[^\n]*)\n', xml)
    (out/'model_bumps.xml').write_text(xml.replace(slope.group(0), slope.group(0)+'\n'.join(bar_lines)+'\n', 1))
    for mesh in meshes:
        shutil.copy(mesh, out/Path(mesh).name)
    subprocess.run(['blender', '--background', str(exp/'output'/stage/car['blend']), '--python-exit-code', '1',
                    '--python', str(WEB/'export_glb.py'), '--', str(out/'car.glb')], check=True, stdout=subprocess.DEVNULL)
    engine = json.loads((exp/'engine.json').read_text())
    vehicle = json.loads((exp/'vehicle.json').read_text())
    params = json.loads((exp/'parameters.json').read_text())

    def control(c):
        if not c:
            return None
        return dict(label=c['label'], detail=c['detail'],
                    devices=[dict(kind=d['kind'], capacity=engine[d['capacity_key']]) for d in c['devices']])
    suspension = None
    if car.get('suspension'):
        suspension = {}
        for axle, a in car['suspension'].items():
            lever = geometry['suspension'][a['lever']] if a['lever'] else 1.0
            first = vehicle['suspension'][a['joints'][0]]
            suspension[axle] = dict(joints=a['joints'], lever_m=lever, sign=a['sign'],
                                    design_rate_kn=round(first['stiffness']/lever**2/1000, 2),
                                    design_damping_kns=round(first['damping']/lever**2/1000, 3))
    com = vehicle['chassis_com']
    ref = subprocess.run([sys.executable, '-c', PARITY.format(exp=str(exp), xml=str(physics/'velo.xml'),
                                                              first=car['parity']['first'], brake=car['parity']['brake'])],
                         check=True, capture_output=True, text=True, cwd=REPO).stdout
    config = dict(
        id=car['id'], name=car['name'], short=car['short'], year=car['year'], controls=car['controls'],
        source_sha256=report['source_sha256'], stage=stage, mass_kg=vehicle['total_mass_kg'],
        engine=dict(rated_power_w=engine['rated_power_w'], governed_rpm=engine['governed_rpm'],
                    no_load_rpm=engine['no_load_rpm'], shift_seconds=engine[car['shift_key']],
                    brake_ramp_seconds=engine['brake_ramp_seconds']),
        gears=[dict(name=n, label=label) for n, label in car['gears']],
        pedal=control(car['pedal']), lever=control(car['lever']), lever_note=car.get('lever_note'),
        declutch_on_pedal=car['declutch_on_pedal'],
        steering=dict(kind=car['steering']['kind'], box_ratio=car['steering']['box_ratio'],
                      limit_deg=geometry['steering']['column_limit_deg']),
        wheelbase=geometry['parameters']['wheelbase'], bodies=list(geometry['bodies']),
        rear_radius=geometry['parameters']['rear_radius'], front_radius=geometry['parameters']['front_radius'],
        seat=[com[0], 0.0, com[2]+.3], suspension=suspension,
        shocks=dict(joints=car['shocks']['joints'], design_nm=params['shock_absorbers']['friction_nm'])
        if car.get('shocks') else None,
        parity=dict(first_gear_at=.5, pedal_at=4.0, seconds=8.0, rows=json.loads(ref)),
        files=sorted(['model_flat.xml', 'model_bumps.xml']+[Path(m).name for m in meshes]))
    (out/'config.json').write_text(json.dumps(config, indent=1)+'\n')
    mujoco.MjModel.from_xml_path(str(physics/'velo.xml'))
    return dict(id=car['id'], short=car['short'], name=car['name'], year=car['year'])


def main():
    SITE.mkdir(parents=True, exist_ok=True)
    bar_lines, bars = course()
    manifest = dict(mujoco_version=mujoco.__version__, bars=bars, cars=[])
    for car in sorted(CARS, key=lambda c: (c['year'], c['id'])):
        print('building', car['id'], flush=True)
        manifest['cars'].append(build_car(car, bar_lines))
    version = mujoco.__version__
    stamp = SITE/'versions.json'
    if not (SITE/'mujoco.wasm').exists() or json.loads(stamp.read_text() if stamp.exists() else '{}').get('mujoco') != version:
        for f in ('mujoco.js', 'mujoco.wasm'):
            urllib.request.urlretrieve(f'https://cdn.jsdelivr.net/npm/@mujoco/mujoco@{version}/{f}', SITE/f)
    stamp.write_text(json.dumps(dict(mujoco=version))+'\n')
    (SITE/'manifest.json').write_text(json.dumps(manifest, indent=1)+'\n')
    write_page()
    total = sum(p.stat().st_size for p in SITE.rglob('*') if p.is_file())
    print(json.dumps(dict(site=str(SITE), cars=[c['id'] for c in manifest['cars']], megabytes=round(total/1e6, 1),
                          wasm_sha256=hashlib.sha256((SITE/'mujoco.wasm').read_bytes()).hexdigest()[:12])))


PAGES = {   # site file: (template, scripts inlined in order)
    'garage.html': ('page.html', ('sim.js', 'input.js', 'app.js')),
    'downhill.html': ('downhill.html', ('sim.js', 'downhill.js')),
    'index.html': ('index.html', ()),
}


def write_page():
    """Each page as a full document; local.html stays as an alias of the garage for old links."""
    for target, (template, scripts) in PAGES.items():
        page = (WEB/template).read_text()
        for name in scripts:
            page = page.replace(f'/*{name}*/', (WEB/name).read_text().replace('export ', ''))
        doc = ('<!doctype html>\n<html lang="en"><head><meta charset="utf-8">'
               '<meta name="viewport" content="width=device-width, initial-scale=1">'
               '</head><body>\n'+page+'\n</body></html>\n')
        (SITE/target).write_text(doc)
    shutil.copy(SITE/'garage.html', SITE/'local.html')


def serve(port):
    import http.server

    class Handler(http.server.SimpleHTTPRequestHandler):
        # Text as UTF-8 explicitly (the page uses − · … and the default guess can be Windows-1252).
        extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map, '.wasm': 'application/wasm',
                          '.html': 'text/html; charset=utf-8', '.json': 'application/json; charset=utf-8',
                          '.xml': 'text/xml; charset=utf-8'}

        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(SITE), **kwargs)
    with http.server.ThreadingHTTPServer(('127.0.0.1', port), Handler) as server:
        print(f'Brass Era Garage: http://localhost:{port}/  (Ctrl+C to stop)', flush=True)
        server.serve_forever()


if __name__ == '__main__':
    if '--page-only' in sys.argv:
        write_page()
    elif '--no-build' not in sys.argv:
        main()
    if '--serve' in sys.argv:
        rest = [a for a in sys.argv[sys.argv.index('--serve')+1:] if a.isdigit()]
        serve(int(rest[0]) if rest else 8321)
