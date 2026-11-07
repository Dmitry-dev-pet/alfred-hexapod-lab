#!/usr/bin/env python3
"""Repair installed Android Hexapod Lab UI + local update server, preserving models."""
from pathlib import Path
import json, os, re, runpy, shutil, signal, socket, subprocess, sys, tempfile, time
import urllib.error, urllib.request

REPO = "Dmitry-dev-pet/alfred-hexapod-lab"
BRANCH = "feature/android-local-offline-viewer-20261104"
BRANCH_API = f"https://api.github.com/repos/{REPO}/branches/{BRANCH}"
BASE = ""  # Filled with a single verified commit SHA at the start of repair.
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
    global BASE
    if not (DEST / "OFFLINE_MANIFEST.json").is_file():
        raise Blocked("Нет установленной сборки: " + str(DEST) + " — ничего не удалено")
    if not (DEST / "vendor/three/build/three.module.js").is_file():
        raise Blocked("Нет локальных файлов Three.js, нужна диагностика установки")
    # Pin all downloaded source files to the same current commit.
    try:
        request = urllib.request.Request(
            BRANCH_API, headers={"User-Agent": "HexapodLab-repair/2"})
        with urllib.request.urlopen(request, timeout=25) as response:
            branch = json.loads(response.read(100_000))
        commit = branch.get("commit", {}).get("sha", "")
        if not re.fullmatch(r"[0-9a-f]{40}", commit):
            raise Blocked("GitHub не вернул проверенный SHA ветки")
        BASE = f"https://raw.githubusercontent.com/{REPO}/{commit}/"
    except urllib.error.URLError as exc:
        raise Blocked("GitHub недоступен: " + str(exc)) from exc
    print("Установлю интерфейс из commit", commit[:12], flush=True)
    # Fetch all sources before touching any installed files.
    with tempfile.TemporaryDirectory(prefix="hexapod-ui-repair-") as tmp:
        root = Path(tmp)
        print("Скачиваю проверенный интерфейс и сервер...", flush=True)
        for rel in SOURCES:
            target = root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(fetch(rel))
        shell = (root / "docs/hexapod/player-shell.js").read_bytes()
        if b"q(\'.hx-library-link\').addEventListener(\'click\',open)" not in shell or b"location.replace(last)" in shell or b"const libraryView=path===\'/hexapod/\'" not in shell:
            raise Blocked("Загружен неправильный дизайн, изменения отменены")
        server = (root / "tools/local_viewer_server.py").read_bytes()
        if any(required not in server for required in
               (b"/api/refresh", b"/api/library", b"/api/refresh-status")):
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
    # A successful catalog refresh is not proof that the browser receives the
    # updated viewer. Check static HTML and JS through the *running* server.
    req = urllib.request.Request(LOCAL + "/hexapod/player-shell.js?verify=20261107a",
                                 headers={"Cache-Control": "no-cache"})
    with urllib.request.urlopen(req, timeout=10) as response:
        served_shell = response.read(200_000)
    installed_shell = (DEST / "hexapod/player-shell.js").read_bytes()
    if served_shell != installed_shell or b"location.replace(last)" in served_shell or (
            b"const libraryView=path==='/hexapod/'" not in served_shell):
        raise Blocked("Сервер отдаёт устаревший player-shell.js — пришли этот текст")
    req = urllib.request.Request(LOCAL + "/hexapod/?library=1&verify=20261107a",
                                 headers={"Cache-Control": "no-cache"})
    with urllib.request.urlopen(req, timeout=10) as response:
        served_home = response.read(200_000)
    if b"/hexapod/player-shell.js?v=20261107b" not in served_home:
        raise Blocked("Сервер отдаёт устаревшую главную страницу — пришли этот текст")
    print("Сервер подтвердил новый интерфейс и отключённый авто-возврат", flush=True)
    before = api("/api/library")
    progress = api("/api/refresh-status")
    if "phase" not in progress:
        raise Blocked("Установленный сервер не поддерживает статус синхронизации")
    revision = before.get("revision", "?")
    print("УСПЕХ: локальная библиотека v%s, %s симуляций" %
          (revision, len(before.get("entries", []))), flush=True)
    print("Файлы моделей и экспериментальные записи сохранены.", flush=True)
    print("Открой " + LOCAL + "/hexapod/?update=20261107b", flush=True)
    print("Теперь нажми «Обновить библиотеку». При ошибке увидишь этап и описание.",
          flush=True)

if __name__ == "__main__":
    try:
        main()
    except (Blocked, ValueError, OSError, urllib.error.URLError, subprocess.TimeoutExpired) as exc:
        print("REPAIR_BLOCKED:", exc, file=sys.stderr)
        print("Модели STL и физические записи не удалялись.", file=sys.stderr)
        sys.exit(1)
