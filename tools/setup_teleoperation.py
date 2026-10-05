#!/usr/bin/env python3
"""Create the local simulation environment, then install its requirements."""

from pathlib import Path
import subprocess
import sys
import venv


def main():
    root = Path(__file__).resolve().parents[1]
    environment = root / '.venv' / 'teleoperation'
    # Data drives may disallow symlinks. venv --copies still normally creates
    # a lib64 symlink on Linux; an existing directory avoids that operation.
    if sys.platform.startswith('linux'):
        (environment / 'lib64').mkdir(parents=True, exist_ok=True)
    venv.EnvBuilder(with_pip=True, symlinks=False).create(environment)
    python = environment / ('Scripts/python.exe' if sys.platform == 'win32' else 'bin/python')
    subprocess.run([str(python), '-m', 'pip', 'install', '-r',
                    str(root / 'requirements-teleoperation.txt')], check=True)
    print('Ready. Start the launcher, then Data → Teleoperation Pipeline.')


if __name__ == '__main__':
    main()
