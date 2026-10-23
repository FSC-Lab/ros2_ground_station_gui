'''
MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
"""Per-machine settings from scripts/config/<name>.conf.

Same KEY="value" files as fsc_PegasusSimulator and fsc_autopilot_ros2 use, read here
rather than sourced, so values are literal (no $VAR expansion). The config is chosen:

  1. FSC_GS_MACHINE, a name in scripts/config/ or a path to a .conf file;
  2. otherwise the config whose HOSTNAME equals this machine's hostname;
  3. otherwise DEFAULT_CONFIG, with a warning naming how to pick one.

The station never refuses to start over this: the settings only feed the LLM tab, and
a cosmetic feature must not stop a flight tool from launching.
"""
import os
import socket

CONFIG_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts', 'config'))
DEFAULT_CONFIG = 'fsc_lab_machine'
ENV_VAR = 'FSC_GS_MACHINE'
REQUIRED_KEYS = ('LLM_BASE_URL', 'LLM_MODEL')


def parse_conf(path):
    """KEY="value" lines; blank lines and # comments are skipped."""
    values = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, value = line.split('=', 1)
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in '"\'':
                value = value[1:-1]
            values[key.strip()] = value
    return values


def _resolve(name):
    if os.path.isfile(name):
        return name
    candidate = os.path.join(CONFIG_DIR, name if name.endswith('.conf') else name + '.conf')
    return candidate if os.path.isfile(candidate) else None


def _all_configs():
    if not os.path.isdir(CONFIG_DIR):
        return []
    return sorted(os.path.join(CONFIG_DIR, f) for f in os.listdir(CONFIG_DIR)
                  if f.endswith('.conf'))


def _select():
    """(path, how it was chosen); path is None if nothing could be found."""
    requested = os.environ.get(ENV_VAR, '').strip()
    if requested:
        path = _resolve(requested)
        if path:
            return path, f'{ENV_VAR}={requested}'
        print(f"[Config] WARNING: {ENV_VAR}={requested} not found in {CONFIG_DIR}; "
              "falling back to hostname matching.")
    host = socket.gethostname()
    for path in _all_configs():
        if parse_conf(path).get('HOSTNAME') == host:
            return path, f'hostname {host}'
    path = _resolve(DEFAULT_CONFIG)
    print(f"[Config] WARNING: no config in {CONFIG_DIR} has HOSTNAME=\"{host}\"; "
          f"using {DEFAULT_CONFIG}. Set HOSTNAME in this machine's .conf or launch "
          f"with {ENV_VAR}=<name>.")
    return path, 'default'


def load():
    """The selected config as a dict, plus '_path' and '_how' for display."""
    path, how = _select()
    values = parse_conf(path) if path else {}
    missing = [k for k in REQUIRED_KEYS if not values.get(k)]
    if missing:
        print(f"[Config] WARNING: {path or CONFIG_DIR} does not set {', '.join(missing)}; "
              "the LLM tab will not reach a server.")
    values['_path'] = path or ''
    values['_how'] = how
    print(f"[Config] Ground station using {path or '(no config)'} ({how}): "
          f"LLM {values.get('LLM_BASE_URL', '?')} / {values.get('LLM_MODEL', '?')}")
    return values
