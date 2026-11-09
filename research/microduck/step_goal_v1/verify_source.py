"""Verify pinned helper Git blobs after reversing only workspace adapters."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent


def verify():
    lock = json.loads((HERE / "source-lock.json").read_text())
    observed = {}
    for path in sorted((HERE / "source").glob("*.py")):
        raw = path.read_bytes()
        text = raw.decode()
        if path.name == "collision_model.py":
            text = text.replace("import sys,contextlib,io,os", "import sys,contextlib,io")
            text = text.replace("WORK=Path(os.environ['MICRODUCK_WORKSPACE']).resolve()\nimport wall_cpu as old",
                                "WORK=next(p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file())\nsys.path.insert(0,str(WORK/'microduck-wall'));import wall_cpu as old")
        if path.name == "measure_clearance.py":
            text = text.replace("import sys,json,contextlib,io,os", "import sys,json,contextlib,io")
            text = text.replace("WORK=Path(os.environ['MICRODUCK_WORKSPACE']).resolve()\nimport fall_cpu as f",
                                "WORK=next(p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file())\nsys.path.insert(0,str(WORK/'microduck-fall'));import fall_cpu as f")
        original = text.encode()
        git_sha = hashlib.sha1(f"blob {len(original)}\0".encode() + original).hexdigest()
        assert git_sha == lock["git_blob_sha1"][path.name], path.name
        observed[path.name] = {"pinned_git_blob_sha1": git_sha, "adapter_sha256": hashlib.sha256(raw).hexdigest()}
    assert len(observed) == 7
    return {"pinned_commit": lock["commit"], "verified_helpers": observed,
            "adapter_scope": "filesystem resolution only; no physics function modifications"}


if __name__ == "__main__":
    result = verify()
    (HERE / "source-verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print("Seven helper Git blobs verified, including both reversed path adapters.")
