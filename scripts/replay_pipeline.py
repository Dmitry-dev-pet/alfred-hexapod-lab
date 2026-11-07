#!/usr/bin/env python3
"""Run a recorded-physics experiment and publish an immutable Android replay.

Requires an explicit, versioned experiment spec and an exporter that writes real
recorded states. No physics is simulated by this module or by the Android viewer.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from register_viewer_replays import register  # noqa: E402

SCHEMA = "hexapod-replay-job/v1"
ID = re.compile(r"[a-z0-9-]{1,64}\Z")
SUFFIXES = {".html", ".js", ".css", ".json"}
VIEWER_BRANCH = "feature/android-local-offline-viewer-20261104"
CATALOG = "docs/hexapod/library.json"
MAX_BYTES = 8_000_000


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def rel_path(value: str, *, prefix: str = "") -> Path:
    require(isinstance(value, str) and 0 < len(value) <= 180, "Invalid path")
    require("\\" not in value and ":" not in value and not value.startswith("/"), "Unsafe path")
    parts = value.split("/")
    require(all(p and p not in (".", "..") and not p.startswith(".") for p in parts), "Unsafe path")
    require(not prefix or value.startswith(prefix), "Unexpected path prefix")
    return Path(*parts)


def inside(root: Path, name: str, *, prefix: str = "") -> Path:
    target = (root / rel_path(name, prefix=prefix)).resolve()
    require(target.is_relative_to(root.resolve()), "Path escapes repository")
    require(not (root / rel_path(name)).is_symlink(), "Symlink asset refused")
    return target


def bytes_of(path: Path, limit: int = MAX_BYTES) -> bytes:
    require(path.is_file() and not path.is_symlink(), "Missing regular file: " + str(path))
    require(0 < path.stat().st_size <= limit, "Invalid file size: " + str(path))
    return path.read_bytes()


def digest(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def dotted(value: dict, key: str):
    require(isinstance(key, str) and key and all(p.isidentifier() for p in key.split(".")),
            "Invalid JSON field selector")
    for p in key.split("."):
        require(isinstance(value, dict) and p in value, "Missing JSON field: " + key)
        value = value[p]
    return value


def number(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def load_spec(filename: Path) -> dict:
    data = filename.read_text(encoding="utf-8")
    if filename.suffix in (".yaml", ".yml"):
        try:
            import yaml  # optional: only for YAML specs, JSON needs no extras
        except ImportError as exc:
            raise RuntimeError("YAML specs need PyYAML; alternatively use JSON") from exc
        job = yaml.safe_load(data)
    else:
        job = json.loads(data)
    require(isinstance(job, dict) and job.get("schema") == SCHEMA, "Unknown job schema")
    sid = job.get("id")
    require(isinstance(sid, str) and ID.fullmatch(sid), "Invalid immutable ID")
    for key, maxlen in (("title", 100), ("detail", 180), ("robot", 100), ("engine", 60)):
        require(isinstance(job.get(key), str) and 0 < len(job[key]) <= maxlen, "Invalid " + key)
    require(job.get("status") in ("passed", "failed", "research"), "Invalid result status")
    require(number(job.get("duration_s")) and 0 < job["duration_s"] <= 3600, "Invalid duration")
    steps = job.get("commands")
    require(isinstance(steps, dict), "Missing simulation/export commands")
    for step in ("simulate", "export"):
        command = steps.get(step)
        require(isinstance(command, list) and len(command) > 0 and
                all(isinstance(s, str) and s and "\x00" not in s for s in command),
                "Missing/unsafe command: " + step)
    trace, replay, player = job.get("trace"), job.get("replay"), job.get("player")
    require(all(isinstance(x, dict) for x in (trace, replay, player)), "Incomplete replay contract")
    inside(ROOT, trace.get("path", ""), prefix="results/")
    require(all(isinstance(trace.get(k), str) and trace[k] for k in
                ("samples_field", "time_field", "qpos_field")), "Incomplete raw-trace selectors")
    rp = replay.get("path")
    require(rp == "hexapod/data/" + sid + ".json", "Replay must have an immutable per-ID JSON path")
    require(isinstance(replay.get("format"), str) and replay["format"], "Replay format required")
    require(isinstance(replay.get("trace_sha256_field"), str), "Trace digest selector required")
    require(player.get("template") != sid and isinstance(player.get("template"), str) and
            ID.fullmatch(player["template"]), "Invalid renderer template")
    repl = player.get("replacements", {})
    require(isinstance(repl, dict) and all(isinstance(k, str) and k and isinstance(v, str)
                                           for k, v in repl.items()), "Invalid renderer substitutions")
    return job


def verify_trace(root: Path, job: dict) -> str:
    spec = job["trace"]
    raw = bytes_of(inside(root, spec["path"], prefix="results/"), limit=100_000_000)
    trace = json.loads(raw)
    samples = dotted(trace, spec["samples_field"])
    require(isinstance(samples, list) and len(samples) >= 2, "No recorded physical samples")
    prev = None
    first = None
    for sample in samples:
        require(isinstance(sample, dict), "Non-object raw physics sample")
        t, q = sample.get(spec["time_field"]), sample.get(spec["qpos_field"])
        require(number(t) and (prev is None or t > prev), "Non-monotone physics time")
        require(isinstance(q, list) and len(q) >= 3 and all(number(x) for x in q),
                "Missing/non-finite physical qpos")
        if first is None:
            first = t
        prev = t
    require(abs((prev - first) - job["duration_s"]) <= max(0.15, job["duration_s"] * 0.15),
            "Physical trace duration differs from replay")
    return digest(raw)


def verify_replay(root: Path, job: dict, trace_sha256: str) -> None:
    spec = job["replay"]
    raw = bytes_of(inside(root / "docs", spec["path"], prefix="hexapod/"))
    obj = json.loads(raw)
    require(isinstance(obj, dict), "Replay must be a JSON object")
    require(obj.get("schema", obj.get("format")) == spec["format"], "Wrong replay format")
    require(number(obj.get("duration_s")) and abs(obj["duration_s"] - job["duration_s"]) < 0.05,
            "Replay duration differs from experiment")
    frames = dotted(obj, spec.get("frames_field", "frames"))
    require(isinstance(frames, list) and len(frames) >= 2, "Empty replay frames")
    require(all(isinstance(frame, (list, dict)) and len(frame) > 0 for frame in frames),
            "Invalid recorded replay frames")
    require(len(frames) <= 200_000, "Excessive replay sample count")
    require(dotted(obj, spec["trace_sha256_field"]) == trace_sha256,
            "Replay not bound to recorded physical trace SHA-256")
    if "frame_count" in obj:
        require(obj["frame_count"] == len(frames), "Replay frame count mismatch")
    engine = obj.get("physics_engine", obj.get("engine"))
    if engine is not None:
        require(isinstance(engine, str) and engine.lower() == job["engine"].lower(),
                "Replay physics engine mismatch")


def make_pages(root: Path, job: dict) -> dict[str, bytes]:
    sid = job["id"]
    template = job["player"]["template"]
    source = inside(root / "docs", "hexapod/" + template + "/index.html")
    require(source.is_file(), "Renderer template has no index.html")
    folder = source.parent
    paths = sorted(p for p in folder.iterdir() if p.is_file() and not p.is_symlink())
    require(0 < len(paths) <= 18, "Renderer template is unexpectedly large")
    output = {}
    changed = False
    seen_replacements = {key: False for key in job["player"].get("replacements", {})}
    for path in paths:
        require(path.suffix in (".html", ".js", ".css"), "Binary/unsupported renderer template")
        body = bytes_of(path).decode("utf-8")
        replaced = body.replace(template, sid)
        changed |= replaced != body
        for old, new in job["player"].get("replacements", {}).items():
            if old in replaced:
                seen_replacements[old] = True
                replaced = replaced.replace(old, new)
        dest = "hexapod/" + sid + "/" + path.name
        output[dest] = replaced.encode("utf-8")
    require(all(seen_replacements.values()), "Some renderer replacement markers were missing")
    require(changed, "Renderer template does not reference its original ID")
    wanted = "/" + job["replay"]["path"]
    require(any(wanted in payload.decode("utf-8") for payload in output.values()),
            "Renderer does not reference the new replay JSON")
    return output


def package(root: Path, job: dict) -> tuple[dict, list[str]]:
    trace_sha = verify_trace(root, job)
    verify_replay(root, job, trace_sha)
    pages = make_pages(root, job)
    files = dict(pages)
    replay_name = job["replay"]["path"]
    files[replay_name] = bytes_of(inside(root / "docs", replay_name, prefix="hexapod/"))
    for name in job.get("extra_assets", []):
        path = rel_path(name, prefix="hexapod/")
        require(path.suffix in SUFFIXES and name not in files, "Invalid/duplicate additional asset")
        files[name] = bytes_of(inside(root / "docs", name))
    require(len(files) <= 20, "Too many assets for a single replay")
    paths_added = []
    try:
        for name, content in pages.items():
            destination = inside(root / "docs", name, prefix="hexapod/")
            if destination.exists():
                require(bytes_of(destination) == content, "Immutable renderer already differs: " + name)
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(content)
            paths_added.append(destination)
        assets = [{"path": name, "sha256": digest(payload), "size": len(payload)}
                  for name, payload in sorted(files.items())]
        entry = {"id": job["id"], "title": job["title"], "detail": job["detail"],
                 "duration_s": job["duration_s"], "href": "/hexapod/" + job["id"] + "/",
                 "status": job["status"], "files": assets}
        catalog_path = root / CATALOG
        catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
        updated = register(catalog, [entry], root / "docs")
        if updated != catalog:
            fd, tmp = tempfile.mkstemp(prefix=".replay-catalog-", dir=catalog_path.parent)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    json.dump(updated, f, ensure_ascii=False, indent=2)
                    f.write("\n")
                os.replace(tmp, catalog_path)
            finally:
                if os.path.exists(tmp):
                    os.unlink(tmp)
        changed_files = [str(p.relative_to(root)) for p in paths_added]
        changed_files.append("docs/" + replay_name)
        changed_files.append(CATALOG)
        return {"id": job["id"], "revision": updated["revision"],
                "added": len(updated["entries"]) - len(catalog["entries"]),
                "trace_sha256": trace_sha, "files": assets}, changed_files
    except Exception:
        for path in paths_added:
            path.unlink(missing_ok=True)
        raise


def git(root: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


def publish(root: Path, paths: list[str], sid: str) -> None:
    require(git(root, "rev-parse", "--show-toplevel") == str(root.resolve()),
            "Not running inside the intended Git working tree")
    require(git(root, "branch", "--show-current") == VIEWER_BRANCH,
            "Publishing requires the owner-managed Android viewer branch")
    require(not git(root, "diff", "--cached", "--name-only"),
            "Unrelated staged changes: refusing automated commit")
    remote = git(root, "remote", "get-url", "origin")
    remote = remote.removesuffix(".git").rstrip("/")
    require(remote in ("https://github.com/Dmitry-dev-pet/alfred-hexapod-lab",
                       "git@github.com:Dmitry-dev-pet/alfred-hexapod-lab"),
            "Unexpected Git remote; publish refused")
    subprocess.run(["git", "-C", str(root), "add", "--", *paths], check=True)
    staged = git(root, "diff", "--cached", "--name-only")
    if staged:
        subprocess.run(["git", "-C", str(root), "commit", "-m",
                        "viewer: publish verified replay " + sid], check=True)
    subprocess.run(["git", "-C", str(root), "push", "origin",
                    "HEAD:refs/heads/" + VIEWER_BRANCH], check=True)


def run(root: Path, job: dict, *, skip_execution: bool = False, do_publish: bool = False) -> dict:
    if do_publish:
        require(git(root, "branch", "--show-current") == VIEWER_BRANCH,
                "Switch to the viewer branch before --publish")
    if not skip_execution:
        for phase in ("simulate", "export"):
            subprocess.run(job["commands"][phase], cwd=root, check=True)
    result, paths = package(root, job)
    if do_publish:
        publish(root, paths, job["id"])
    return {**result, "published": do_publish}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=["run"])
    ap.add_argument("spec", type=Path)
    ap.add_argument("--skip-execution", action="store_true", help="Package already computed raw physics")
    ap.add_argument("--publish", action="store_true", help="Commit and push to viewer branch")
    args = ap.parse_args()
    job = load_spec(args.spec)
    print(json.dumps(run(ROOT, job, skip_execution=args.skip_execution,
                         do_publish=args.publish), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
