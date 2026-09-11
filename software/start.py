"""Portable entry points. No robot connection is made during install or init."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys
from urllib.parse import urlencode
import venv
import webbrowser

ROOT = Path(__file__).resolve().parent


def initialize(root: Path = ROOT) -> bool:
    """Create credentials locally; preserve an existing builder's calibration."""
    target = root / 'robot_config.json'
    if target.exists():
        return False
    data = json.loads((root / 'robot_config.example.json').read_text(encoding='utf-8'))
    data.setdefault('network', {})['token'] = secrets.token_urlsafe(32)
    with target.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2)
        stream.write('\n')
    if os.name != 'nt':
        target.chmod(0o600)
    return True


def run_module(module: str, *args: str) -> int:
    return subprocess.call([sys.executable, '-m', module, *args], cwd=ROOT)


def serial_port(prompt: str) -> str:
    port = input(prompt).strip()
    if port and not (re.fullmatch(r'COM[1-9][0-9]*', port, re.I) or port.startswith('/dev/')):
        raise ValueError('Use the verified COM port or /dev/serial/by-id path.')
    return port


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='DogV3 setup and control launchers')
    parser.add_argument('action', choices=('install', 'init', 'commission', 'operate', 'flash', 'check', 'open-pi'))
    args = parser.parse_args(argv)
    os.chdir(ROOT)
    if sys.version_info < (3, 11):
        raise RuntimeError('Install Python 3.11 or newer first.')
    if args.action == 'install':
        directory = ROOT / '.venv'
        interpreter = directory / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
        if not interpreter.exists():
            venv.EnvBuilder(with_pip=True).create(directory)
        subprocess.check_call([str(interpreter), '-m', 'pip', 'install', '-e', '.[commission]'], cwd=ROOT)
        initialize()
        print('Installed. Open Commission-Windows.bat to begin, or Operate-Windows.bat for a dry preview.')
        return 0
    if args.action == 'init':
        print('Created local uncommissioned configuration.' if initialize() else 'Existing configuration preserved.')
        return 0
    initialize()
    if args.action == 'check':
        return run_module('dogv3.setup_program.readiness', '--config', 'robot_config.json', '--require-commissioned', '--strict-warnings')
    if args.action == 'commission':
        print('Support the robot with feet clear. Connect only the verified ESP32. Blank opens a dry preview.')
        port = serial_port('ESP32 serial port: ')
        flags = ['--config', 'robot_config.json']
        if port:
            flags += ['--port-serial', port]
        return run_module('dogv3.setup_program.server', *flags)
    if args.action == 'operate':
        print('1: Dry preview (default)   2: Open onboard Pi   3: USB bench control')
        choice = input('Choice [1]: ').strip() or '1'
        if choice == '2':
            return open_pi()
        if choice == '3':
            port = serial_port('Verified ESP32 serial port: ')
            if not port:
                raise ValueError('A serial port is required for USB operation.')
            return run_module('dogv3.runtime.operator_server', '--port-serial', port, '--config', 'robot_config.json')
        if choice != '1':
            raise ValueError('Choose 1, 2 or 3.')
        return run_module('dogv3.runtime.operator_server', '--dry', '--config', 'robot_config.json')
    if args.action == 'open-pi':
        return open_pi()
    if args.action == 'flash':
        port = serial_port('Verified ESP32 serial port to FLASH: ')
        if not port:
            raise ValueError('An explicit port is required.')
        print('This overwrites firmware on that ESP32. The robot must be supported with its feet clear.')
        if input('Type FLASH to proceed: ').strip() != 'FLASH':
            print('Cancelled.')
            return 0
        return run_module('dogv3.setup_program.flash_test', '--first-flash', '--port', port)
    return 2


def open_pi() -> int:
    network = json.loads((ROOT / 'robot_config.json').read_text(encoding='utf-8'))['network']
    host = network.get('pi_host', 'dogv3.local')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,252}', host):
        raise ValueError('Set network.pi_host to your Pi hostname or IP, without a URL or port.')
    port = int(network.get('operate_port', 8001))
    if not 1 <= port <= 65535:
        raise ValueError('Invalid operator port.')
    if not network.get('token'):
        raise ValueError('Use the same locally generated network token as the commissioned Pi configuration.')
    webbrowser.open(f'http://{host}:{port}/?' + urlencode({'key': network['token']}))
    print('Opened the Pi controls. The Pi must already have your commissioned configuration and services running.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'DogV3: {error}', file=sys.stderr)
        raise SystemExit(1)
