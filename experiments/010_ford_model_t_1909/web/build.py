"""Assemble the Model T web simulation in output/web/ (git-ignored).

.venv/bin/python experiments/010_ford_model_t_1909/web/build.py [--serve [PORT]]

--serve then serves output/web on localhost (default port 8321); open /local.html.

Run after run.py --stage full. Writes:
- model_t_bumps.xml, model_t_flat.xml: the physics model Python validates (make_model with the
  calibrated springs), mesh paths reduced to file names; plus the visual-only OBJ meshes
- model_t.glb: the car's appearance, each part in its MuJoCo body's frame (export_glb.py)
- config.json: engine, driver, levers, seat point and design values, from this car's sources
- mujoco.js, mujoco.wasm: the official MuJoCo 3.14.0 WebAssembly build (pinned, from jsDelivr),
  matching the Python MuJoCo version
- index.html (page content for publishing) and local.html (same, with a doctype, for a local
  server), with sim.js and app.js inlined
"""
import hashlib
import json
import re
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path

import mujoco

WEB = Path(__file__).resolve().parent
CAR = WEB.parent
REPO = CAR.parent.parent
OUT = CAR/'output/web'
MUJOCO_JS = 'https://cdn.jsdelivr.net/npm/@mujoco/mujoco@{v}/{f}'
sys.path.insert(0, str(CAR))


def main():
    import importlib.util
    spec = importlib.util.spec_from_file_location('bumpy', CAR/'bumpy_road.py')
    bumpy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bumpy)
    from control import ENGINE
    OUT.mkdir(parents=True, exist_ok=True)
    physics, geometry, _ = bumpy.geometries()
    ratios = {k: n*ENGINE['axle_ratio'] for k, n in ENGINE['gearbox_ratios'].items()}
    engine = dict(torque_nm=ENGINE['rated_power_w']/(ENGINE['governed_rpm']*2*3.141592653589793/60), ratios=ratios)
    meshes = set()
    for name, extra in (('bumps', bumpy.course()), ('flat', ())):
        bumpy.vehicle.make_model(geometry, physics, 0, engine=engine, extra=extra, path_name=f'web_{name}.xml')
        xml = (physics/f'web_{name}.xml').read_text()
        meshes |= set(re.findall(r'file="([^"]+\.obj)"', xml))
        xml = re.sub(r'file="([^"]+)/([^"/]+\.obj)"', r'file="\2"', xml)
        (OUT/f'model_t_{name}.xml').write_text(xml)
        mujoco.MjModel.from_xml_path(str(physics/f'web_{name}.xml'))   # the unmodified model compiles
    for mesh in meshes:
        shutil.copy(mesh, OUT/Path(mesh).name)
    subprocess.run(['blender', '--background', str(CAR/'output/full/model_t.blend'), '--python-exit-code', '1',
                    '--python', str(WEB/'export_glb.py'), '--', str(OUT/'model_t.glb')],
                   check=True, stdout=subprocess.DEVNULL)
    version = mujoco.__version__
    for f in ('mujoco.js', 'mujoco.wasm'):
        target = OUT/f
        if not target.exists() or json.loads((OUT/'versions.json').read_text() if (OUT/'versions.json').exists()
                                             else '{}').get('mujoco') != version:
            urllib.request.urlretrieve(MUJOCO_JS.format(v=version, f=f), target)
    (OUT/'versions.json').write_text(json.dumps(dict(mujoco=version))+'\n')
    sus = geometry['suspension']
    config = dict(
        source_sha256=geometry['source_sha256'], mujoco_version=version, engine=ENGINE,
        wheelbase=geometry['parameters']['wheelbase'], preview_s=.8,
        levers=dict(front=sus['front_radius_rod_m'], rear=sus['radius_rod_m']),
        seat=bumpy.SEAT.tolist(), bars=[dict(x=x, height=h, side=s) for x, h, s in bumpy.BARS],
        bodies=list(geometry['bodies']),
        files=sorted(['model_t_bumps.xml', 'model_t_flat.xml']+[Path(m).name for m in meshes]))
    (OUT/'config.json').write_text(json.dumps(config, indent=1)+'\n')
    if not (WEB/'page.html').exists():
        print('no page.html yet: physics files only')
        return
    page = (WEB/'page.html').read_text()
    page = page.replace('/*SIM_JS*/', (WEB/'sim.js').read_text().replace('export ', ''))
    page = page.replace('/*APP_JS*/', (WEB/'app.js').read_text())
    (OUT/'index.html').write_text(page)
    (OUT/'local.html').write_text('<!doctype html>\n<html lang="en"><head><meta charset="utf-8">'
                                  '<meta name="viewport" content="width=device-width, initial-scale=1">'
                                  '</head><body>\n'+page+'\n</body></html>\n')
    sizes = {p.name: p.stat().st_size for p in sorted(OUT.iterdir())}
    print(json.dumps(dict(out=str(OUT), bytes=sum(sizes.values()), files=sizes,
                          wasm_sha256=hashlib.sha256((OUT/'mujoco.wasm').read_bytes()).hexdigest()[:12]), indent=1))


def serve(port):
    import http.server

    class Handler(http.server.SimpleHTTPRequestHandler):
        extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map, '.wasm': 'application/wasm'}

        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(OUT), **kwargs)
    with http.server.ThreadingHTTPServer(('127.0.0.1', port), Handler) as server:
        print(f'Model T Spring Lab: http://localhost:{port}/local.html  (Ctrl+C to stop)', flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
    if '--serve' in sys.argv:
        rest = sys.argv[sys.argv.index('--serve')+1:]
        serve(int(rest[0]) if rest else 8321)
