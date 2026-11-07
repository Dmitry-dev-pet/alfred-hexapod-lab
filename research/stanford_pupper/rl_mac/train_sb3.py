#!/usr/bin/env python3
"""Train Pupper v3 with CPU MuJoCo + Gymnasium + SB3 PPO (no Mac runner creation)."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['smoke','train'],default='smoke')
    parser.add_argument('--device',choices=['cpu','mps'],default='cpu')
    parser.add_argument('--num-envs',type=int,default=4)
    parser.add_argument('--steps',type=int,default=None)
    parser.add_argument('--seed',type=int,default=20261107)
    parser.add_argument('--profile',choices=['tracking','stability'],default='tracking')
    parser.add_argument('--out',type=Path,default=Path.home()/'alfred-robotics'/'pupper-m4-train')
    args=parser.parse_args()
    if not 1<=args.num_envs<=16:parser.error('--num-envs must be 1..16')
    steps=args.steps if args.steps is not None else (4096 if args.mode=='smoke' else 100000)
    if steps<1024 or steps>2000000:parser.error('--steps outside accepted bounds')
    import torch
    import mujoco
    import numpy as np
    import gymnasium
    import stable_baselines3 as sb3
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
    from gym_env import PupperVelocityGym,REWARD_PROFILES
    from physics import ORIGINAL_XML,git_blob_sha,XML_BLOB

    if args.device=='mps' and not torch.backends.mps.is_available():
        parser.error('MPS requested but torch.backends.mps.is_available() is false')
    torch.set_num_threads(1)
    os.environ.setdefault('OMP_NUM_THREADS','1')
    os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
    args.out.mkdir(parents=True,exist_ok=True)
    if any(args.out.iterdir()):
        parser.error('The output directory is not empty; use a new unique --out path')
    env=DummyVecEnv([lambda i=i: PupperVelocityGym(profile=args.profile,seed=args.seed+i)
                     for i in range(args.num_envs)])
    started=time.monotonic()
    try:
        policy_kwargs={'net_arch':{'pi':[256,128,128,128],'vf':[256,128,128,128]},
                       'activation_fn':torch.nn.ELU}
        model=PPO('MlpPolicy',env,policy_kwargs=policy_kwargs,
                  learning_rate=3e-4,n_steps=256,batch_size=256,n_epochs=4,
                  gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=.005,
                  seed=args.seed,device=args.device,verbose=1)
        model.learn(total_timesteps=steps,progress_bar=False)
        checkpoint=args.out/'policy_final.zip'
        model.save(str(checkpoint))
        elapsed=time.monotonic()-started
        outcome={
            'schema':'alfred-pupper-sb3-m4-training-receipt/v1',
            'status':'TRAINED_CHECKPOINT_CREATED_NOT_YET_EVALUATED',
            'platform':platform.platform(),'machine':platform.machine(),
            'python':sys.version.split()[0],'torch':torch.__version__,
            'numpy':np.__version__,'mujoco':mujoco.__version__,
            'gymnasium':gymnasium.__version__,'sb3':sb3.__version__,
            'device_requested':args.device,'device_used':str(model.device),
            'num_envs':args.num_envs,'seed':args.seed,'profile':args.profile,
            'reward_weights':REWARD_PROFILES[args.profile],
            'total_timesteps_requested':steps,'total_timesteps_executed':model.num_timesteps,
            'seconds_training_wall':round(elapsed,3),'steps_per_second':round(model.num_timesteps/elapsed,3),
            'policy_checkpoint_name':checkpoint.name,
            'policy_checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
            'upstream_mjcf_git_blob_sha1':git_blob_sha(ORIGINAL_XML.read_bytes()),
            'upstream_mjcf_expected_sha1':XML_BLOB,
            'model_architecture':'SB3 MlpPolicy 960 x 256-128-128-128 x 12; NOT Stanford MJLab PPO',
            'limits':['Single deterministic training run is not proof of locomotion',
                      'This is an independently authored Gymnasium wrapper, not original MJLab training',
                      'Not compatible with original Stanford deploy policy.json without exporter/parity',
                      'Never control a real robot with this checkpoint.'],
        }
        (args.out/'receipt.json').write_text(json.dumps(outcome,indent=2,ensure_ascii=False)+'\n')
        print('TRAINING_CHECKPOINT_CREATED',checkpoint,'receipt',args.out/'receipt.json',flush=True)
    finally:
        env.close()


if __name__=='__main__':main()
