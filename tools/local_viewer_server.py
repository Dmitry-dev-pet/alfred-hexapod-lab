#!/usr/bin/env python3
"""Local-only Hexapod replay server with on-demand, verified library updates.

No daemon, no periodic polls. Browser POST /api/refresh checks one owner-managed
GitHub branch and downloads manifest-listed assets only after SHA-256 checks.
"""
import argparse
import hashlib
import http.server
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PID_FILE = ROOT / ".local_viewer_server.pid"
HOST, PORT = "127.0.0.1", 8808
URL = "http://127.0.0.1:8808/hexapod/"
LIBRARY = ROOT / "hexapod" / "library.json"
REPO = "Dmitry-dev-pet/alfred-hexapod-lab"
BRANCH = "feature/android-local-offline-viewer-20261104"
BRANCH_API = "https://api.github.com/repos/" + REPO + "/branches/" + BRANCH
RAW_BASE = "https://raw.githubusercontent.com/" + REPO + "/"
HEAD_RE = re.compile(r"^[0-9a-f]{40}$")
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
ID_RE = re.compile(r"^[a-z0-9-]{1,64}$")
HREF_RE = re.compile(r"^/hexapod/(?:[a-z0-9-]+/)?$")
REFRESH_LOCK = threading.Lock()
REFRESH_STATUS_LOCK = threading.Lock()
REFRESH_STATUS = {"phase": "idle", "checked": 0, "total": 0, "asset": "", "error": ""}


def refresh_status(phase=None, **fields):
    """A read-only progress snapshot for the browser while the POST is running."""
    with REFRESH_STATUS_LOCK:
        if phase is not None:
            REFRESH_STATUS["phase"] = phase
        REFRESH_STATUS.update(fields)
        return dict(REFRESH_STATUS)


ALLOWED_SUFFIX = {".json", ".html", ".css", ".js"}
MAX_FILE_BYTES = 8_000_000
MAX_ENTRIES = 120


def remote_bytes(url, limit):
    req = urllib.request.Request(
        url, headers={"User-Agent": "Alfred-Hexapod-Local-Viewer/1",
                      "Accept": "application/json" if url == BRANCH_API else "*/*"})
    with urllib.request.urlopen(req, timeout=30) as response:
        if response.status != 200:
            raise ValueError("Unexpected remote status")
        body = response.read(limit + 1)
        if len(body) > limit:
            raise ValueError("Remote file exceeds size limit")
        return body


def safe_asset_path(name):
    """Allow only immutable frontend files below this viewer's hexapod directory."""
    if not isinstance(name, str) or not name.startswith("hexapod/"):
        raise ValueError("Invalid asset prefix")
    parts = name.split("/")
    if any(p in ("", ".", "..") or p.startswith(".") for p in parts):
        raise ValueError("Invalid asset path")
    if len(parts) > 8 or len(name) > 180 or "\\" in name or ":" in name:
        raise ValueError("Invalid asset name")
    if Path(name).suffix not in ALLOWED_SUFFIX:
        raise ValueError("Unsupported asset type")
    dest = (ROOT / name).resolve()
    if not dest.is_relative_to((ROOT / "hexapod").resolve()):
        raise ValueError("Asset path escapes viewer")
    return dest


def validate_catalog(obj):
    if not isinstance(obj, dict) or obj.get("schema") != "hexapod-library/v1":
        raise ValueError("Unsupported library schema")
    revision = obj.get("revision")
    entries = obj.get("entries")
    if type(revision) is not int or revision < 1:
        raise ValueError("Invalid catalog revision")
    if not isinstance(entries, list) or not (1 <= len(entries) <= MAX_ENTRIES):
        raise ValueError("Invalid library size")
    ids, hrefs = set(), set()
    for item in entries:
        if not isinstance(item, dict):
            raise ValueError("Invalid simulation entry")
        sim_id, href = item.get("id"), item.get("href")
        if not isinstance(sim_id, str) or not ID_RE.fullmatch(sim_id) or sim_id in ids:
            raise ValueError("Invalid or duplicate simulation ID")
        if not isinstance(href, str) or not HREF_RE.fullmatch(href) or href in hrefs:
            raise ValueError("Invalid or duplicate simulation route")
        ids.add(sim_id)
        hrefs.add(href)
        if not isinstance(item.get("title"), str) or not (1 <= len(item["title"]) <= 100):
            raise ValueError("Invalid simulation title")
        if not isinstance(item.get("detail"), str) or len(item["detail"]) > 180:
            raise ValueError("Invalid simulation detail")
        if not isinstance(item.get("duration_s"), (int, float)) or not (0 < item["duration_s"] <= 3600):
            raise ValueError("Invalid simulation duration")
        files = item.get("files", [])
        if not isinstance(files, list) or len(files) > 20:
            raise ValueError("Invalid simulation file list")
        seen = set()
        for f in files:
            if not isinstance(f, dict) or not isinstance(f.get("path"), str):
                raise ValueError("Invalid file entry")
            path = f["path"]
            safe_asset_path(path)
            if path in seen:
                raise ValueError("Duplicate file path")
            seen.add(path)
            if not isinstance(f.get("sha256"), str) or not SHA_RE.fullmatch(f["sha256"]):
                raise ValueError("Missing file SHA-256")
            size = f.get("size")
            if type(size) is not int or not (1 <= size <= MAX_FILE_BYTES):
                raise ValueError("Invalid file size")
    return obj


def read_local_library():
    return validate_catalog(json.loads(LIBRARY.read_text(encoding="utf-8")))


def refresh_library():
    """No background work. A single explicit browser click performs the refresh."""
    local = read_local_library()
    refresh_status("github", checked=0, total=0, asset="", error="")
    branch = json.loads(remote_bytes(BRANCH_API, 100_000))
    sha = branch.get("commit", {}).get("sha", "")
    if not HEAD_RE.fullmatch(sha):
        raise ValueError("GitHub branch did not return a verified commit ID")
    base = RAW_BASE + sha + "/docs/"
    remote = validate_catalog(json.loads(remote_bytes(base + "hexapod/library.json", 200_000)))
    if remote["revision"] < local["revision"]:
        raise ValueError("Remote library revision is older than local library")
    file_total = sum(len(e.get("files", [])) for e in remote["entries"])
    checked = 0
    refresh_status("files", checked=checked, total=file_total)
    local_ids = {e["id"] for e in local["entries"]}
    remote_ids = {e["id"] for e in remote["entries"]}
    if not local_ids.issubset(remote_ids):
        raise ValueError("Remote catalog unexpectedly removes installed experiments")
    installed = 0
    local_file_index = {f["path"]: f for entry in local["entries"] for f in entry.get("files", [])}
    with tempfile.TemporaryDirectory(prefix=".hexapod-sync-", dir=ROOT) as temp_name:
        stage = Path(temp_name)
        staged = []
        for entry in remote["entries"]:
            for file in entry.get("files", []):
                destination = safe_asset_path(file["path"])
                refresh_status("files", checked=checked, total=file_total,
                               asset=file["path"])
                wanted_sha, expected_size = file["sha256"], file["size"]
                if destination.exists():
                    if not destination.is_file() or destination.is_symlink():
                        raise ValueError("Existing asset is not a regular file")
                    existing = hashlib.sha256(destination.read_bytes()).hexdigest()
                    if existing != wanted_sha:
                        cached = local_file_index.get(file["path"], {})
                        if cached.get("sha256") != wanted_sha or cached.get("local_sha256") != existing:
                            raise ValueError("Immutable local asset differs: " + file["path"])
                        file["local_sha256"] = existing
                    checked += 1
                    refresh_status("files", checked=checked, total=file_total, asset="")
                    continue
                if any(previous[1] == destination for previous in staged):
                    raise ValueError("Duplicate asset across simulations")
                url = base + urllib.parse.quote(file["path"], safe="/")
                refresh_status("download", checked=checked, total=file_total, asset=file["path"])
                payload = remote_bytes(url, expected_size)
                if len(payload) != expected_size or hashlib.sha256(payload).hexdigest() != wanted_sha:
                    raise ValueError("Remote asset integrity mismatch: " + file["path"])
                # New MuJoCo CAD recordings originally reference upstream STL.
                # Repoint them to the 30 verified local meshes for true offline replay.
                if destination.name.startswith("cad-rig-") and destination.suffix == ".json":
                    rig = json.loads(payload)
                    if rig.get("format") != "fnk0052-mujoco-cad-visual-replay-v1":
                        raise ValueError("Unsupported CAD replay format")
                    meshes = rig.get("meshes", [])
                    if len(meshes) != 30:
                        raise ValueError("New replay needs the complete CAD30 mesh set")
                    originals = json.loads((ROOT / "OFFLINE_MANIFEST.json").read_text())
                    digests = originals.get("files_sha256", {})
                    for mesh in meshes:
                        filename = mesh.get("filename", "")
                        if not re.fullmatch(r"[a-z0-9_-]+\.stl", filename):
                            raise ValueError("Invalid CAD mesh filename")
                        local_mesh = ROOT / "assets" / "stl" / filename
                        expected = digests.get("assets/stl/" + filename)
                        if not expected or not local_mesh.is_file():
                            raise ValueError("Missing verified local CAD mesh")
                        if hashlib.sha256(local_mesh.read_bytes()).hexdigest() != expected:
                            raise ValueError("Local CAD mesh integrity mismatch")
                        mesh["url"] = "/assets/stl/" + filename
                    payload = json.dumps(rig, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
                # Existing Three.js import maps in new pages are switched to the
                # same vetted local copies that the offline installer provides.
                if destination.suffix == ".html":
                    payload = payload.replace(
                        b"https://cdn.jsdelivr.net/npm/three@0.169.0/build/three.module.js",
                        b"/vendor/three/build/three.module.js").replace(
                        b"https://cdn.jsdelivr.net/npm/three@0.169.0/examples/jsm/",
                        b"/vendor/three/examples/jsm/")
                if destination.suffix == ".js" and re.search(rb"https?://[^\s'\"]+\.(?:js|json|stl)", payload):
                    raise ValueError("Remote asset references from executable script")
                file["local_sha256"] = hashlib.sha256(payload).hexdigest()
                staging = stage / file["path"]
                staging.parent.mkdir(parents=True, exist_ok=True)
                staging.write_bytes(payload)
                staged.append((staging, destination))
                checked += 1
                refresh_status("files", checked=checked, total=file_total, asset="")
        refresh_status("install", checked=checked, total=file_total, asset="")
        # Never advertise a new simulation unless its page and all assets exist.
        staged_destinations = {destination for _, destination in staged}
        for item in remote["entries"]:
            page = safe_asset_path(item["href"].lstrip("/") + "index.html")
            if not (page.exists() or page in staged_destinations):
                raise ValueError("Missing player page for: " + item["id"])
        for staging, destination in staged:
            destination.parent.mkdir(parents=True, exist_ok=True)
            if destination.exists():
                raise ValueError("Concurrent file write")
            os.replace(staging, destination)
            installed += 1
        data = json.dumps(remote, ensure_ascii=False, indent=2).encode("utf-8") + b"\n"
        tmp_library = stage / "library.json"
        tmp_library.write_bytes(data)
        os.replace(tmp_library, LIBRARY)
    return {"added": len(remote_ids - local_ids), "assets": installed,
            "revision": remote["revision"], "source_commit": sha}


def is_ours(pid):
    if not isinstance(pid, int) or pid < 2:
        return False
    try:
        os.kill(pid, 0)
        cmd = (Path("/proc") / str(pid) / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        return "local_viewer_server.py" in cmd and "--serve" in cmd
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
        with urllib.request.urlopen(URL, timeout=1.0) as response:
            return response.status == 200
    except Exception:
        return False


def run_server():
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(ROOT), **kwargs)

        def log_message(self, fmt, *args):
            pass

        def send_json(self, value, code=200):
            data = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if urllib.parse.urlsplit(self.path).path == "/api/refresh-status":
                self.send_json(refresh_status())
                return
            if urllib.parse.urlsplit(self.path).path == "/api/library":
                try:
                    self.send_json(read_local_library())
                except Exception as ex:
                    self.send_json({"error": str(ex)}, 500)
                return
            super().do_GET()

        def do_POST(self):
            if urllib.parse.urlsplit(self.path).path != "/api/refresh":
                self.send_json({"error": "Unknown action"}, 404)
                return
            host = self.headers.get("Host", "")
            origin = self.headers.get("Origin", "")
            if (host not in ("127.0.0.1:8808", "localhost:8808") or
                origin not in ("", "http://127.0.0.1:8808", "http://localhost:8808") or
                self.headers.get("X-Hexapod-Action") != "refresh"):
                self.send_json({"error": "Same-origin action required"}, 403)
                return
            if self.headers.get("Content-Length", "0") != "0":
                self.send_json({"error": "Request body not accepted"}, 400)
                return
            if not REFRESH_LOCK.acquire(blocking=False):
                self.send_json({"error": "Обновление уже выполняется"}, 409)
                return
            refresh_status("github", checked=0, total=0, asset="", error="")
            try:
                result = refresh_library()
                refresh_status("done", asset="")
                self.send_json(result)
            except Exception as ex:
                refresh_status("error", asset="", error=str(ex))
                print("Hexapod refresh failed:", type(ex).__name__, str(ex),
                      file=sys.stderr, flush=True)
                self.send_json({"error": "Обновление не выполнено: " + str(ex)}, 502)
            finally:
                REFRESH_LOCK.release()

    with http.server.ThreadingHTTPServer((HOST, PORT), Handler) as server:
        server.serve_forever(poll_interval=.2)


def start():
    pid = prior_pid()
    if pid:
        print("ALREADY_RUNNING", URL)
        return
    with socket.socket() as sock:
        if sock.connect_ex((HOST, PORT)) == 0:
            raise SystemExit("BLOCKED: port 8808 already in use")
    with (ROOT / "local_viewer.log").open("a") as log:
        proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--serve"],
                                cwd=str(ROOT), stdin=subprocess.DEVNULL,
                                stdout=log, stderr=log, start_new_session=True, close_fds=True)
    PID_FILE.write_text(str(proc.pid))
    for _ in range(40):
        if not is_ours(proc.pid):
            break
        if http_ok():
            print("RUNNING", URL)
            return
        time.sleep(.2)
    raise SystemExit("FAILED: server did not answer; check local_viewer.log")


def stop():
    pid = prior_pid()
    if not pid:
        print("NOT_RUNNING")
        return
    os.kill(pid, signal.SIGTERM)
    for _ in range(20):
        if not is_ours(pid):
            break
        time.sleep(.1)
    if not is_ours(pid):
        PID_FILE.unlink(missing_ok=True)
        print("STOPPED")
    else:
        print("STOP_SENT")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", nargs="?", choices=("start", "stop", "status"))
    parser.add_argument("--serve", action="store_true")
    args = parser.parse_args()
    if args.serve:
        run_server()
    elif args.action == "start":
        start()
    elif args.action == "stop":
        stop()
    elif args.action == "status":
        print("RUNNING " + URL if prior_pid() and http_ok() else "NOT_RUNNING")
    else:
        parser.print_help()
