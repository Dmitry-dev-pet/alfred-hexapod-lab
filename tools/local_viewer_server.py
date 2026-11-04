#!/usr/bin/env python3
"""Start/stop private localhost-only static viewer; no new Termux services."""
import argparse
import http.server
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PID_FILE = ROOT / '.local_viewer_server.pid'
HOST, PORT = '127.0.0.1', 8808
URL = f'http://{HOST}:{PORT}/hexapod/'


def is_ours(pid):
    if not isinstance(pid, int) or pid < 2:
        return False
    try:
        os.kill(pid, 0)
        cmd = (Path('/proc') / str(pid) / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        return 'local_viewer_server.py' in cmd and '--serve' in cmd
    except (OSError, PermissionError):
        return False


def prior_pid():
    try:
        pid = int(PID_FILE.read_text().strip())
    except (ValueError, OSError):
        return None
    return pid if is_ours(pid) else None


def http_ok():
    try:
        with urllib.request.urlopen(URL, timeout=1.0) as r:
            return r.status == 200
    except Exception:
        return False


def run_server():
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(ROOT), **kwargs)
        def log_message(self, fmt, *args):
            pass
    with http.server.ThreadingHTTPServer((HOST, PORT), Handler) as server:
        server.serve_forever(poll_interval=.2)


def start():
    pid = prior_pid()
    if pid:
        print('ALREADY_RUNNING', PID_FILE, URL)
        return
    with socket.socket() as s:
        if s.connect_ex((HOST, PORT)) == 0:
            raise SystemExit('BLOCKED: port 8808 already in use')
    with (ROOT / 'local_viewer.log').open('a') as log:
        proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--serve'],
                                cwd=str(ROOT), stdin=subprocess.DEVNULL,
                                stdout=log, stderr=log, start_new_session=True, close_fds=True)
    PID_FILE.write_text(str(proc.pid))
    for _ in range(40):
        if not is_ours(proc.pid):
            break
        if http_ok():
            print('RUNNING', URL)
            return
        time.sleep(.2)
    raise SystemExit('FAILED: server did not answer, check local_viewer.log')


def stop():
    pid = prior_pid()
    if not pid:
        print('NOT_RUNNING')
        return
    os.kill(pid, signal.SIGTERM)
    for _ in range(20):
        if not is_ours(pid):
            break
        time.sleep(.1)
    if not is_ours(pid):
        PID_FILE.unlink(missing_ok=True)
        print('STOPPED')
    else:
        print('STOP_SENT')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', nargs='?', choices=('start', 'stop', 'status'))
    parser.add_argument('--serve', action='store_true')
    args = parser.parse_args()
    if args.serve:
        run_server()
    elif args.action == 'start':
        start()
    elif args.action == 'stop':
        stop()
    elif args.action == 'status':
        print('RUNNING ' + URL if prior_pid() and http_ok() else 'NOT_RUNNING')
    else:
        parser.print_help()
