"""G2 — immutable Stanford Pupper v3 MJLab/RSL-RL T4 scaling + bounded PPO.

Runs on a single owner-authorized Google Colab T4 VM via Colab CLI.
- Benchmark 16, 64 and 256 envs, 8 PPO iterations each, 24 steps/env/iter.
- If at least one baseline passes, run up to 128 further iterations using the
  highest measured warm iteration environment throughput (or wall-rate fallback).
- Never run a CPU fallback or alter upstream Stanford sources.
- Always package logs and receipts, even when an individual run fails.
- The invoking Mac workflow downloads results before stopping the Colab VM.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time
import zipfile

SOURCE_SHA = "aa475e9a393a209b29fd5de7e0ee10f3de643230"
LOCK_SHA = "2bcba14e27f84d1f746f45cd10b16a2790ba369ebac922b6a8bc290012409f01"
REPO = "https://github.com/cs123-stanford/pupper-mjlab.git"
TASK = "Mjlab-VelocityFS-Flat-Pupper-v3"
SEED = 20261107
BENCH_ENVS = (16, 64, 256)
BENCH_ITERATIONS = 8
EXTENDED_ITERATIONS = 128
STEPS_PER_ENV = 24
# This is an experiment cap, not the Colab VM timeout. Fail closed on overrun.
BENCH_ELAPSED_BUDGET_S = 540
EXTENDED_SUBPROCESS_TIMEOUT_S = 780
ROOT = Path("/content/stanford-original-g2")
SRC = ROOT / "source"
RESULT = ROOT / "result"
BUNDLE = Path("/content/pupper-g2-results.zip")

REWARDS = {
    "track_linear_velocity": 3.0,
    "track_yaw_velocity": 1.5,
    "upright": 1.0,
    "base_height": 0.20,
    "termination": -10.0,
    "action_rate_l2": -0.05,
}

# This worker uses the ORIGINAL upstream code, not an independently reimplemented
# RL algorithm. It is launched in Stanford's pinned uv/venv once per test case.
WORKER = r'''
from __future__ import annotations
import hashlib, json, os, sys, time
from pathlib import Path
import torch, warp, mujoco, mujoco_warp
import mjlab.tasks
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
from mjlab.scripts.train import TrainConfig, launch_training

n, iterations, save_every, label = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
assert label in ("bench16", "bench64", "bench256", "extended")
assert n in (16, 64, 256) and 0 < iterations <= 128
assert os.environ.get("CUDA_VISIBLE_DEVICES") == "0", "No CUDA→CPU fallback"
assert torch.cuda.is_available() and torch.cuda.device_count() >= 1, "Missing CUDA"
assert (torch.tensor([7.0],device="cuda:0")*2).item() == 14.0
device = torch.cuda.get_device_name(0)

task = "Mjlab-VelocityFS-Flat-Pupper-v3"
root = Path("/content/stanford-original-g2/result/runs") / label
root.mkdir(parents=True,exist_ok=False)
env = load_env_cfg(task)
agent = load_rl_cfg(task)
env.scene.num_envs = n
reward_weights = {
  "track_linear_velocity":3.0,"track_yaw_velocity":1.5,
  "upright":1.0,"base_height":0.2,"termination":-10.0,
  "action_rate_l2":-0.05,
}
for name, value in reward_weights.items():
    assert name in env.rewards and env.rewards[name].weight == 0, "Upstream task differs: "+name
    env.rewards[name].weight = value
agent.seed = 20261107
agent.logger = "tensorboard"
agent.upload_model = False
agent.max_iterations = iterations
agent.num_steps_per_env = 24
agent.save_interval = save_every
agent.experiment_name = "stanford_native_cuda_g2_" + label
agent.run_name = label
protocol = {
    "schema":"pupper-g2-native-rslrl-case-v1",
    "upstream_source_sha":"aa475e9a393a209b29fd5de7e0ee10f3de643230",
    "task":task,
    "label":label,
    "gpu_name":device,
    "torch_version":str(torch.__version__),
    "torch_cuda_build":str(torch.version.cuda),
    "mujoco_version":str(mujoco.__version__),
    "warp_version":str(warp.__version__),
    "envs":n,"iterations":iterations,"steps_per_env":24,
    "transitions":n*iterations*24,"seed":20261107,
    "rewards":reward_weights,"wandb":"disabled","backend":"native RSL-RL",
}
(root/"protocol.json").write_text(json.dumps(protocol,indent=2)+"\n")
print("G2_NATIVE_TRAIN_BEGIN", label, device, flush=True)
start=time.monotonic()
launch_training(task_id=task, args=TrainConfig(env=env, agent=agent, gpu_ids=[0], log_root=str(root/"logs")))
elapsed=time.monotonic()-start
checkpoints=sorted((root/"logs").rglob("model_*.pt"),
                   key=lambda path:int(path.stem.rsplit("_",1)[-1]))
assert checkpoints, "Original RSL-RL did not save a checkpoint"
models=[]
for path in checkpoints:
    blob=path.read_bytes()
    assert len(blob)>100000
    models.append({"path":str(path.relative_to(root)),"bytes":len(blob),"sha256":hashlib.sha256(blob).hexdigest()})
evidence={
    "status":"NATIVE_CUDA_RSLRL_CHECKPOINTS_PASS",
    "case":protocol,
    "training_wall_seconds":round(elapsed,6),
    "checkpoints":models,
    "max_torch_cuda_memory_allocated_bytes":torch.cuda.max_memory_allocated(0),
    "max_torch_cuda_memory_reserved_bytes":torch.cuda.max_memory_reserved(0),
}
(root/"worker_receipt.json").write_text(json.dumps(evidence,indent=2)+"\n")
print("G2_NATIVE_TRAIN_COMPLETE",label,len(models),round(elapsed,3),flush=True)
'''


def sha_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def logged(args: list[str], label: str, *, cwd: Path | None = None,
           timeout_s: int = 180, extra_env: dict[str,str] | None = None) -> float:
    log = RESULT / (label + ".log")
    started = time.monotonic()
    with log.open("w", encoding="utf-8") as fp:
        env = os.environ | {"UV_NO_PROGRESS":"1","WANDB_MODE":"disabled","WANDB_DISABLED":"true"}
        if extra_env:
            env.update(extra_env)
        process = subprocess.run(args, cwd=str(cwd) if cwd else None,
                                 stdout=fp,stderr=subprocess.STDOUT,
                                 env=env, timeout=timeout_s,check=False)
    if process.returncode:
        raise RuntimeError(f"stage_{label}_exit_{process.returncode}")
    return time.monotonic()-started


def iteration_seconds(logtext: str) -> list[float]:
    # RSL-RL prints per-iteration wall time. Sampled first iterations include
    # compilation/warmup, so summarize both overall and post-warmup numbers.
    pats = [r"Iteration time\s*:\s*([0-9]+(?:\.[0-9]+)?)",
            r"Iteration time\s*=\s*([0-9]+(?:\.[0-9]+)?)"]
    for pat in pats:
        found = [float(v) for v in re.findall(pat,logtext,re.IGNORECASE)]
        if found:
            return found
    return []


def retain_metrics(label: str) -> dict[str,object]:
    # TensorBoard metrics are optional. A missing tag must never be fabricated.
    run_root = RESULT / "runs" / label
    path = run_root / "metrics.json"
    curves: dict[str,object] = {"scalar_tags":[],"scalars":{}}
    try:
        from tensorboard.backend.event_processing import event_accumulator
        for event in sorted(run_root.rglob("events.out.tfevents.*")):
            reader = event_accumulator.EventAccumulator(str(event),
                    size_guidance={"scalars":0})
            reader.Reload()
            for tag in reader.Tags().get("scalars",[]):
                if tag not in curves["scalars"] and len(curves["scalars"]) < 80:
                    data=reader.Scalars(tag)
                    curves["scalars"][tag] = [
                        {"step":int(e.step),"value":float(e.value)} for e in data
                    ]
        curves["scalar_tags"]=sorted(curves["scalars"].keys())
    except Exception as exc:
        curves["metrics_parse_error_type"]=type(exc).__name__
    path.write_text(json.dumps(curves,indent=2)+"\n")
    return {"scalar_tag_count":len(curves["scalar_tags"]),
            "metrics_error_type":curves.get("metrics_parse_error_type")}


def make_archive(manifest: dict[str,object]) -> None:
    # Small transferable evidence; all binaries have SHA-256 in receipts.
    manifest["driver_sha_note"] = "script supplied verbatim by pinned GitHub source"
    (RESULT/"summary.json").write_text(json.dumps(manifest,indent=2)+"\n")
    with zipfile.ZipFile(BUNDLE, "w",compression=zipfile.ZIP_DEFLATED,compresslevel=3) as z:
        for path in sorted(RESULT.rglob("*")):
            if not path.is_file():
                continue
            rel=str(path.relative_to(RESULT))
            if path.suffix in (".json",".yaml",".log",".txt",".pt"):
                z.write(path,rel)


def run_case(python: Path, label: str, n: int, iterations: int,
             save_interval: int, timeout_s: int) -> dict[str,object]:
    env={"CUDA_VISIBLE_DEVICES":"0","OMP_NUM_THREADS":"1"}
    wall=logged([str(python),str(ROOT/"native_case.py"),str(n),
                 str(iterations),str(save_interval),label],
                label+"_training",cwd=SRC,timeout_s=timeout_s,extra_env=env)
    root=RESULT/"runs"/label
    receipt=json.loads((root/"worker_receipt.json").read_text())
    if receipt["status"]!="NATIVE_CUDA_RSLRL_CHECKPOINTS_PASS":
        raise RuntimeError(f"invalid_native_receipt_{label}")
    for model in receipt["checkpoints"]:
        path=root/model["path"]
        if sha_bytes(path.read_bytes())!=model["sha256"]:
            raise RuntimeError(f"checkpoint_sha_mismatch_{label}")
    log=(RESULT/(label+"_training.log")).read_text(errors="replace")
    iterations_reported=iteration_seconds(log)
    warm_samples=iterations_reported[2:]
    warmed_avg = sum(warm_samples)/len(warm_samples) if warm_samples else None
    warm_steps_per_s = round(n*STEPS_PER_ENV/warmed_avg,3) if warmed_avg else None
    metrics=retain_metrics(label)
    # Only preserve selected immutable checkpoints in the transport archive;
    # retain actual complete list of native saved checkpoints in the receipt.
    models=receipt["checkpoints"]
    keep={models[0]["path"],models[-1]["path"]}
    if label=="extended" and len(models)>2:
        keep.add(models[len(models)//2]["path"])
    # Preserve the full original checkpoint manifest; explicitly identify
    # the subset shipped in this bounded transport ZIP.
    receipt["transport_checkpoints"]=[
        model for model in models if model["path"] in keep
    ]
    (root/"worker_receipt.json").write_text(json.dumps(receipt,indent=2)+"\n")
    for model in models:
        if model["path"] not in keep:
            (root/model["path"]).unlink()
    return {
        "status":"PASS","name":label,"envs":n,"iterations":iterations,
        "environment_steps":n*iterations*STEPS_PER_ENV,
        "train_wall_seconds":round(wall,4),
        "native_train_wall_seconds":receipt["training_wall_seconds"],
        "wall_environment_steps_per_s":round(n*iterations*STEPS_PER_ENV/wall,3),
        "native_iteration_seconds":iterations_reported,
        "warm_iteration_seconds_avg":round(warmed_avg,6) if warmed_avg else None,
        "warm_environment_steps_per_s":warm_steps_per_s,
        "gpu_name":receipt["case"]["gpu_name"],
        "torch_cuda_build":receipt["case"]["torch_cuda_build"],
        "gpu_torch_mem_reserved":receipt.get("max_torch_cuda_memory_reserved_bytes"),
        "checkpoint_count_original":len(models),
        "checkpoint_count_transport":len(keep),
        "tensorboard":metrics,
    }


def main() -> int:
    start=time.monotonic()
    if ROOT.exists() or BUNDLE.exists():
        print("G2_REFUSED_EXISTING_RESULT_PATH",flush=True)
        return 3
    RESULT.mkdir(parents=True)
    summary: dict[str,object] = {
        "schema":"stanford-g2-native-cuda-v1",
        "stage":"initial",
        "source_sha":SOURCE_SHA,"task":TASK,
        "seed":SEED,"reward_weights":REWARDS,
        "bench_envs":BENCH_ENVS,"bench_iterations":BENCH_ITERATIONS,
        "extended_iterations_target":EXTENDED_ITERATIONS,
        "results":[],"extended":None,"no_cpu_fallback":True,
        "status":"IN_PROGRESS","start_os":platform.platform(),
        "start_python":sys.version.split()[0],
    }
    exit_status=2
    try:
        import torch
        summary["start_torch"]=str(torch.__version__)
        summary["start_torch_cuda_build"]=str(torch.version.cuda)
        summary["stage"]="gpu_preflight"
        if not torch.cuda.is_available() or torch.cuda.device_count()<1:
            raise RuntimeError("CUDA_GPU_REQUIRED_NO_CPU_FALLBACK")
        if (torch.tensor([5.0],device="cuda:0")+3.0).item()!=8.0:
            raise RuntimeError("GPU_ARITHMETIC_FAILED")
        summary["gpu"]=torch.cuda.get_device_name(0)
        summary["gpu_total_memory_bytes"]=torch.cuda.get_device_properties(0).total_memory

        summary["stage"]="pinned_stanford_checkout"
        SRC.mkdir()
        logged(["git","init","-q",str(SRC)],"git_init",timeout_s=30)
        logged(["git","remote","add","origin",REPO],"git_remote",cwd=SRC,timeout_s=30)
        logged(["git","fetch","--depth=1","origin",SOURCE_SHA],"git_fetch",cwd=SRC,timeout_s=120)
        logged(["git","checkout","-q","--detach","FETCH_HEAD"],"git_checkout",cwd=SRC,timeout_s=45)
        if subprocess.check_output(["git","rev-parse","HEAD"],cwd=SRC,text=True).strip()!=SOURCE_SHA:
            raise RuntimeError("SOURCE_COMMIT_SHA_MISMATCH")
        lock = SRC/"uv.lock"
        if sha_bytes(lock.read_bytes())!=LOCK_SHA:
            raise RuntimeError("ORIGINAL_STANFORD_LOCK_CHANGED")
        summary["uv_lock_sha256"]=LOCK_SHA

        summary["stage"]="pinned_dependency_install"
        logged([sys.executable,"-m","pip","install","--quiet","uv==0.9.18"],
               "install_uv",timeout_s=180)
        uv = shutil.which("uv") or str(Path(sys.executable).parent/"uv")
        if not Path(uv).is_file():
            raise RuntimeError("UV_NOT_FOUND_AFTER_INSTALL")
        summary["uv_version"]=subprocess.check_output([uv,"--version"],text=True).strip()
        logged([uv,"sync","--locked","--no-dev","--extra","cu128",
                "--python",sys.executable,"--no-progress"],
               "uv_sync_cuda",cwd=SRC,timeout_s=720)
        env_python=SRC/".venv/bin/python"
        if not env_python.is_file():
            raise RuntimeError("VENV_NOT_INSTALLED")
        (ROOT/"native_case.py").write_text(WORKER)

        summary["stage"]="three_size_benchmark"
        for n in BENCH_ENVS:
            label=f"bench{n}"
            try:
                out=run_case(env_python,label,n,BENCH_ITERATIONS,
                             save_interval=4,timeout_s=240)
                summary["results"].append(out)
                print("G2_BENCH_CASE_PASS",label,
                      out["warm_environment_steps_per_s"],
                      out["wall_environment_steps_per_s"],flush=True)
            except Exception as exc:
                summary["results"].append({"status":"FAIL","name":label,
                                          "envs":n,"error_type":type(exc).__name__,
                                          "error_label":str(exc)[:100]})
                print("G2_BENCH_CASE_FAIL",label,type(exc).__name__,flush=True)
            if time.monotonic()-start>BENCH_ELAPSED_BUDGET_S:
                summary["bench_time_budget_exceeded"]=True
                break

        passed=[x for x in summary["results"] if x["status"]=="PASS"]
        summary["benchmark_pass_count"]=len(passed)
        if not passed:
            raise RuntimeError("NO_BENCHMARK_CONFIGURATION_PASSED")
        if len(passed)!=len(BENCH_ENVS):
            summary["benchmark_partial"]=True
        # Prefer steady iteration throughput (less contaminated by compilation).
        selected=max(passed,key=lambda x:x["warm_environment_steps_per_s"]
                     if x["warm_environment_steps_per_s"] is not None
                     else x["wall_environment_steps_per_s"])
        summary["selected_num_envs"]=selected["envs"]
        summary["selection_rule"]="highest_measured_warm_env_steps_per_s"

        summary["stage"]="extended_training"
        if time.monotonic()-start>BENCH_ELAPSED_BUDGET_S:
            summary["extended"]={"status":"SKIPPED","reason":"benchmark_elapsed_budget_exceeded"}
            summary["status"]="BENCH_ONLY_EXT_SKIPPED"
            exit_status=0
        else:
            try:
                ext=run_case(env_python,"extended",selected["envs"],
                             EXTENDED_ITERATIONS,save_interval=32,
                             timeout_s=EXTENDED_SUBPROCESS_TIMEOUT_S)
                summary["extended"]=ext
                summary["status"]="G2_BENCH_AND_EXTENDED_NATIVE_CUDA_PASS"
                print("G2_EXTENDED_NATIVE_CUDA_PASS",
                      ext["envs"],ext["environment_steps"],flush=True)
                exit_status=0
            except Exception as exc:
                summary["extended"]={"status":"FAIL","error_type":type(exc).__name__,
                                     "error_label":str(exc)[:100]}
                summary["status"]="BENCH_PASS_EXT_FAILED"
                exit_status=4
    except Exception as exc:
        summary["status"]="G2_FATAL_FAIL"
        summary["error_type"]=type(exc).__name__
        summary["error_label"]=str(exc)[:130] if isinstance(exc,RuntimeError) else ""
        exit_status=5
    finally:
        summary["stage_completed"]=summary["stage"]
        summary["total_driver_wall_seconds"]=round(time.monotonic()-start,3)
        make_archive(summary)
        print("G2_RESULT_ZIP_READY",BUNDLE.name,BUNDLE.stat().st_size,flush=True)
        print("G2_RESULT_STATUS",summary["status"],flush=True)
    return exit_status


if __name__=="__main__":
    raise SystemExit(main())
