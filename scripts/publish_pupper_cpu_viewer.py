#!/usr/bin/env python3
"""Publish verified Stanford Pupper physical replay into existing tablet catalog.

No physics, training or browser assets created here. Only lists assets already
committed to this branch, after SHA256 byte counts and replay contract checks.
"""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
CATALOG=ROOT/"docs/hexapod/library.json"
PATHS=(
    "hexapod/pupper-stand-cpu-v1/index.html",
    "hexapod/pupper-stand-cpu-v1/app.js",
    "hexapod/data/pupper-v3-cpu-stance-5s-v1.json",
)
NEW_ID="pupper-v3-cpu-stand-v1"


def main():
    catalog=json.loads(CATALOG.read_text(encoding="utf8"))
    assert catalog["schema"]=="hexapod-library/v1"
    entries=catalog["entries"]
    assert len({e["id"] for e in entries})==len(entries)
    if any(e["id"]==NEW_ID for e in entries):
        found=next(e for e in entries if e["id"]==NEW_ID)
        assert catalog["revision"]>=27 and found["href"]=="/hexapod/pupper-stand-cpu-v1/"
        print("PUPPER_CATALOG_ALREADY_CURRENT",catalog["revision"])
        return
    assert catalog["revision"]==26 and len(entries)==39, "Unexpected current viewer catalog: inspect before retrying"
    assert all(e["href"]!="/hexapod/pupper-stand-cpu-v1/" for e in entries)
    asset_files=[]
    for name in PATHS:
        path=ROOT/"docs"/name
        payload=path.read_bytes()
        assert 1<=len(payload)<=8_000_000, name
        assert path.suffix in (".js",".json",".html"), name
        asset_files.append({
            "path":name, "sha256":hashlib.sha256(payload).hexdigest(),
            "size":len(payload),
        })
    html=(ROOT/"docs"/PATHS[0]).read_text(encoding="utf8")
    js=(ROOT/"docs"/PATHS[1]).read_text(encoding="utf8")
    data=json.loads((ROOT/"docs"/PATHS[2]).read_text(encoding="utf8"))
    assert html.count('src="/hexapod/player-shell.js?v=20261107a"')==1
    assert html.count('src="/hexapod/pupper-stand-cpu-v1/app.js"')==1
    assert html.count("https://cdn.jsdelivr.net/npm/three@0.169.0/build/three.module.js")==1
    assert html.count("https://cdn.jsdelivr.net/npm/three@0.169.0/examples/jsm/")==1
    assert 'class="layout"' in html and 'class="stage"' in html
    assert 'id="play"' in html and 'id="seek"' in html and 'id="speed"' in html
    assert 'id="viewer"' in html and 'id="reset-camera"' in html
    assert "pupper-v3-mujoco-cpu-stance-replay-v1" in js
    assert "https://raw.githubusercontent.com/G-Levine/pupper_v3_description/" in js
    assert data["schema"]=="pupper-v3-mujoco-cpu-stance-replay-v1" and not data["is_walk"]
    assert data["physics_steps"]==1250 and data["duration_s"]==5.0
    assert data["layout"]["stride"]==109
    assert len(data["frames"])==250 and len(data["visual_meshes"])==13
    assert all(len(f)==109 for f in data["frames"])
    assert data["frames"][-1][108]==4
    assert data["source"]["original_xml_git_blob_sha1"]=="7771e176c42b3aed1d48cc62434771b0fceb7c8c"
    assert data["source"]["visual_source_prefix"] in js
    entry={
        "id":NEW_ID,
        "title":"Stanford Pupper · стойка CPU",
        "detail":"5 с · настоящая MuJoCo-физика, 1250 шагов · 13 оригинальных STL · без обучения нейросети",
        "duration_s":5.0,
        "href":"/hexapod/pupper-stand-cpu-v1/",
        "status":"research",
        "files":asset_files,
    }
    entries.append(entry)
    catalog["revision"]=27
    assert len(entries)==40 and len({x["href"] for x in entries})==40
    CATALOG.write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    print("PUPPER_CATALOG_PUBLISHED",catalog["revision"],len(entries))
    for f in asset_files: print(f["path"],f["size"],f["sha256"])


if __name__=="__main__":
    main()
