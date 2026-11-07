#!/usr/bin/env python3
"""Repair installed Android Hexapod Lab UI + local update server, preserving models."""
from pathlib import Path
import json, os, runpy, shutil, signal, socket, subprocess, sys, tempfile, time
import urllib.error, urllib.request

REPO = "Dmitry-dev-pet/alfred-hexapod-lab"
COMMIT = "df27baadf944be1dfd26b6cb6a49163606c26497"  # direct library bottom-sheet fix + catalog v25
BASE = f"https://raw.githubusercontent.com/{REPO}/{COMMIT}/"
DEST = Path.home() / "hexapod-viewer-local"
LOCAL = "http://127.0.0.1:8808"
UI = (
    "hexapod/index.html", "hexapod/app.js",
    "hexapod/claws/index.html", "hexapod/claws/app.js",
    "hexapod/six-to-four/index.html", "hexapod/six-to-four/app.js",
    "hexapod/player-shell.js", "hexapod/player-shell.css",
)
SOURCES = tuple("docs/" + p for p in UI) + (
    "tools/local_viewer_server.py", "tools/upgrade_android_ui.py",
    "docs/hexapod/library.json",
)

class Blocked(Exception):
    pass

def port_busy():
    with socket.socket() as s:
        s.settimeout(.75)
        return s.connect_ex(("127.0.0.1", 8808)) == 0

def stop_owned_server():
    if not port_busy():
        return
    try:
        pid = int((DEST / ".local_viewer_server.pid").read_text().strip())
        cmdline = (Path("/proc") / str(pid) / "cmdline").read_bytes().replace(b"\x00", b" ").decode(errors="replace")
        if pid < 2 or str(DEST / "local_viewer_server.py") not in cmdline or "--serve" not in cmdline:
            raise ValueError("Not our daemon")
    except (OSError, ValueError) as exc:
        raise Blocked(
            "Порт 8808 занят другим сервером. Останови старый сервер Ctrl+C в его сессии Termux "
            "и повтори команду. Если неясно, пришли: ps -A | grep python"
        ) from exc
    print("Останавливаю только наш локальный viewer:", pid, flush=True)
    os.kill(pid, signal.SIGTERM)
    for _ in range(40):
        if not port_busy():
            return
        time.sleep(.15)
    raise Blocked("Сервер не остановился. Существующие файлы не изменены.")

def fetch(source):
    req = urllib.request.Request(BASE + source, headers={"User-Agent": "HexapodLab-repair/1"})
    with urllib.request.urlopen(req, timeout=35) as response:
        if response.status != 200:
            raise Blocked("Не удалось скачать " + source)
        b = response.read(2_000_001)
    if not (10 <= len(b) <= 2_000_000):
        raise Blocked("Некорректный размер исходника: " + source)
    return b

def api(path, post=False):
    req = urllib.request.Request(
        LOCAL + path, method="POST" if post else "GET",
        headers={"X-Hexapod-Action": "refresh"} if post else {},
        data=b"" if post else None)
    try:
        with urllib.request.urlopen(req, timeout=240 if post else 8) as response:
            return json.loads(response.read(1_000_000))
    except urllib.error.HTTPError as ex:
        body = ex.read(4096).decode(errors="replace")
        try:
            body = json.loads(body).get("error", body)
        except ValueError:
            pass
        raise Blocked("HTTP %s %s: %s" % (ex.code, path, body)) from ex

def main():
    if not (DEST / "OFFLINE_MANIFEST.json").is_file():
        raise Blocked("Нет установленной сборки: " + str(DEST) + " — ничего не удалено")
    if not (DEST / "vendor/three/build/three.module.js").is_file():
        raise Blocked("Нет локальных файлов Three.js, нужна диагностика установки")
    # Fetch all sources before touching any installed files.
    with tempfile.TemporaryDirectory(prefix="hexapod-ui-repair-") as tmp:
        root = Path(tmp)
        print("Скачиваю проверенный интерфейс и сервер...", flush=True)
        for rel in SOURCES:
            target = root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(fetch(rel))
        if b"q(\'.hx-library-link\').addEventListener(\'click\',open)" not in (root / "docs/hexapod/player-shell.js").read_bytes():
            raise Blocked("Загружен неправильный дизайн, изменения отменены")
        server = (root / "tools/local_viewer_server.py").read_bytes()
        if b"/api/refresh" not in server or b"/api/library" not in server:
            raise Blocked("Загруженный сервер не поддерживает библиотеку")
        upgrader = runpy.run_path(str(root / "tools/upgrade_android_ui.py"))["upgrade"]
        stop_owned_server()
        try:
            result = upgrader(root, DEST)
        except Exception:
            # The official upgrader rolls back partial updates; restore previous viewer service.
            subprocess.run([sys.executable, str(DEST / "local_viewer_server.py"), "start"],
                           cwd=DEST, capture_output=True, text=True, timeout=25)
            raise
        print("Резервная копия:", result["backup"], flush=True)
    started = subprocess.run(
        [sys.executable, str(DEST / "local_viewer_server.py"), "start"],
        cwd=DEST, capture_output=True, text=True, timeout=25)
    if started.returncode:
        raise Blocked("Сервер не запустился: " + (started.stderr or started.stdout).strip())
    print(started.stdout.strip(), flush=True)
    before = api("/api/library")
    print("Локальный каталог: v" + str(before.get("revision")), flush=True)
    print("Синхронизирую библиотеку...", flush=True)
    refresh = api("/api/refresh", post=True)
    after = api("/api/library")
    if after.get("revision", 0) < 25 or not any(
            x.get("id") == "lite3-s2-compare-v2" for x in after.get("entries", [])):
        raise Blocked("Библиотека v25 и сравнение Lite3 не подтверждены")
    print("УСПЕХ: v%s, записей %s, новых %s" % (
        after["revision"], len(after["entries"]), refresh.get("added", "?")), flush=True)
    print("Открой: " + LOCAL + "/hexapod/lite3-s2-compare-v2/?v=" +
          str(after["revision"]), flush=True)

if __name__ == "__main__":
    try:
        main()
    except (Blocked, ValueError, OSError, urllib.error.URLError, subprocess.TimeoutExpired) as exc:
        print("REPAIR_BLOCKED:", exc, file=sys.stderr)
        print("Модели STL и физические записи не удалялись.", file=sys.stderr)
        sys.exit(1)
