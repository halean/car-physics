"""Launch Blender using the local Python environment (no third-party packages)."""
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / 'experiments/001_patent_motorwagen'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blender', default='blender')
    parser.add_argument('--config', type=Path, default=EXPERIMENT / 'parameters.json')
    parser.add_argument('--output', type=Path, default=EXPERIMENT / 'output')
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    executable = shutil.which(args.blender)
    if not executable:
        parser.error('Blender not found; use --blender /path/to/blender')
    if not args.config.is_file():
        parser.error(f'Config not found: {args.config}')
    command = [executable, '--background', '--factory-startup', '--python-exit-code', '1',
               '--python', str(EXPERIMENT / 'generate.py'), '--',
               '--config', str(args.config.resolve()), '--output', str(args.output.resolve())]
    if args.render:
        command.append('--render')
    subprocess.run(command, check=True)


if __name__ == '__main__':
    main()
