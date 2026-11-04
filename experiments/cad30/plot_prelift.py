"""Chart exploratory results. No inference from 20 temporally correlated runs."""
from pathlib import Path
import json
import matplotlib.pyplot as plt
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'hexlab').is_dir())
r=json.loads((ROOT/'results/prelift_force_transfer_paired_2026-11-04.json').read_text())
mus=[1.1,.6,.3];g=r['groups']
a=[g[str(mu)]['neighbor_force_gate_supported']['mean_advance_cm'] for mu in mus]
b=[g[str(mu)]['neighbor_pre_lift_transfer']['mean_advance_cm'] for mu in mus]
plt.figure(figsize=(8,4.5));plt.plot(mus,a,'o-',label='Previous controller');plt.plot(mus,b,'s-',label='Unload before liftoff')
plt.xticks(mus,[str(mu) for mu in mus]);plt.xlabel('Assumed friction coefficient (μ)');plt.ylabel('Distance traveled in 8 s (cm)')
plt.title('FNK0052 CAD30: local force transfer during walking');plt.grid(alpha=.25);plt.legend();plt.tight_layout()
p=ROOT/'results/prelift_distance_2026-11-04.png';plt.savefig(p,dpi=145);print(p);plt.close()