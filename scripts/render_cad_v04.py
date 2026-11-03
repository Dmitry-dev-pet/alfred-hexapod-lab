"""Optional CAD-accurate preview from the verified upstream MuJoCo meshes.

Renders our estimated-mass model; not a photographic depiction of a real
assembled robot. The image is a *diagnostic*, not a validated standing pose.

Usage on headless Ubuntu:
  xvfb-run -a python scripts/render_cad_v04.py --out results/cad_v04_preview.png
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hexlab.cad_mujoco_v04 import DEFAULT_CACHE, build


def write_png(path: Path, rgb) -> None:
    """Store a displayable 8-bit RGB PNG using the standard library only."""
    height, width, channels = rgb.shape
    if channels != 3:
        raise ValueError("Expected RGB renderer image")
    def chunk(tag, data):
        return struct.pack(">I",len(data)) + tag + data + struct.pack(">I",zlib.crc32(tag+data)&0xffffffff)
    raw = b"".join(bytes([0]) + row.tobytes() for row in rgb)
    result = (
        bytes.fromhex("89504e470d0a1a0a") +
        chunk(b"IHDR", struct.pack(">IIBBBBB",width,height,8,2,0,0,0)) +
        chunk(b"IDAT", zlib.compress(raw,6)) +
        chunk(b"IEND",b"")
    )
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(result)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--scenario",choices=("nominal","lower","upper"),default="nominal")
    p.add_argument("--cache",type=Path,default=DEFAULT_CACHE)
    p.add_argument("--out",type=Path,default=ROOT/"results/cad_v04_preview.png")
    p.add_argument("--pose-receipt",type=Path,
                   help="Use the candidate qpos from the constrained six-leg stand receipt")
    args=p.parse_args()

    import mujoco

    xml, _receipt = build(args.scenario, cache=args.cache,offline=True)
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    if args.pose_receipt is not None:
        report = json.loads(args.pose_receipt.read_text(encoding="utf-8"))
        if report["scenario"] != args.scenario:
            raise ValueError("Mismatched standing-pose scenario")
        q = report["qpos_candidate"]
        if len(q) != model.nq:
            raise ValueError(f"Expected {model.nq} joint positions, got {len(q)}")
        data.qpos[:] = q
    mujoco.mj_forward(model,data)
    cam=mujoco.MjvCamera()
    cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:]=(0,0,.10 if args.pose_receipt else .15)
    cam.distance=.77 if args.pose_receipt else .85
    cam.azimuth=115
    cam.elevation=-25
    with mujoco.Renderer(model,height=480,width=640) as render:
        render.update_scene(data,camera=cam)
        picture=render.render()
        write_png(args.out,picture)
    print("Preview",args.out,args.out.stat().st_size,"bytes")


if __name__=="__main__":
    main()
