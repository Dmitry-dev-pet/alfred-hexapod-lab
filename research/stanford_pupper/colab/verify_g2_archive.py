"""Independently verify a downloaded G2 Colab ZIP using only Python stdlib.

Exit nonzero unless all three native Stanford throughput tests and the longer
native training run passed, and every *transported* RSL-RL checkpoint matches
the source recorded SHA-256. Does not load or execute model weights.
"""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

SOURCE = "aa475e9a393a209b29fd5de7e0ee10f3de643230"
EXPECTED_ENVS = [16,64,256]
EXPECTED_WEIGHTS = {
    "track_linear_velocity":3.0, "track_yaw_velocity":1.5,
    "upright":1.0,"base_height":0.2,
    "termination":-10.0,"action_rate_l2":-0.05,
}

def verify(archive_path: Path) -> dict:
    assert archive_path.is_file() and archive_path.stat().st_size>1000000
    whole_sha = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    with zipfile.ZipFile(archive_path) as archive:
        assert archive.testzip() is None, "Bad ZIP member CRC"
        summary=json.loads(archive.read("summary.json"))
        assert summary["source_sha"]==SOURCE
        assert summary["seed"]==20261107
        assert summary["reward_weights"]==EXPECTED_WEIGHTS
        assert summary["no_cpu_fallback"] is True
        cases=summary["results"]
        assert [x["envs"] for x in cases]==EXPECTED_ENVS
        assert all(x["status"]=="PASS" for x in cases)
        assert all(x["iterations"]==8 and x["environment_steps"]==x["envs"]*8*24 for x in cases)
        extended=summary["extended"]
        assert extended and extended["status"]=="PASS"
        assert extended["iterations"]==128
        assert extended["environment_steps"]==128*extended["envs"]*24
        assert extended["envs"] in EXPECTED_ENVS
        assert summary["selected_num_envs"]==extended["envs"]
        assert summary["status"]=="G2_BENCH_AND_EXTENDED_NATIVE_CUDA_PASS"
        models=[]
        for case in [*cases,extended]:
            label=case["name"]
            receipt=json.loads(archive.read(f"runs/{label}/worker_receipt.json"))
            assert receipt["status"]=="NATIVE_CUDA_RSLRL_CHECKPOINTS_PASS"
            info=receipt["case"]
            assert info["upstream_source_sha"]==SOURCE
            assert info["task"]=="Mjlab-VelocityFS-Flat-Pupper-v3"
            assert info["rewards"]==EXPECTED_WEIGHTS
            assert info["seed"]==20261107
            assert info["torch_cuda_build"]
            assert info["gpu_name"]
            assert info["envs"]==case["envs"]
            assert info["iterations"]==case["iterations"]
            assert info["transitions"]==case["environment_steps"]
            transported=receipt["transport_checkpoints"]
            assert transported, "No checkpoint was transported"
            for model in transported:
                content=archive.read(f"runs/{label}/{model['path']}")
                assert len(content)==model["bytes"]
                assert hashlib.sha256(content).hexdigest()==model["sha256"]
                models.append({"case":label,"path":model["path"],
                               "bytes":model["bytes"],"sha256":model["sha256"]})
        return {"status":"G2_DOWNLOAD_AND_SHA256_VERIFIED",
                "sha256":whole_sha,"zip_bytes":archive_path.stat().st_size,
                "source":SOURCE,"environment_sizes":EXPECTED_ENVS,
                "selected_envs":extended["envs"],
                "extended_env_steps":extended["environment_steps"],
                "benchmark":cases,"extended":extended,
                "transported_checkpoints":models}

if __name__=="__main__":
    if len(sys.argv)!=3:
        raise SystemExit("usage: verify_g2_archive.py ARCHIVE.zip REPORT.json")
    proof=verify(Path(sys.argv[1]))
    Path(sys.argv[2]).write_text(json.dumps(proof,indent=2)+"\n")
    print("G2_INDEPENDENT_CHECKPOINT_SHA256_PASS",
          len(proof["transported_checkpoints"]))
    print("G2_ARCHIVE_SHA256",proof["sha256"])
    print("G2_NATIVE_CUDA_ENV_SIZES",proof["environment_sizes"])
    print("G2_EXTENDED_ENV_STEPS",proof["extended_env_steps"])
