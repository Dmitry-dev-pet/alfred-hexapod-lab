"""Plot data measured from saved MuJoCo physical trace; no simulated numbers invented."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'hexlab').is_dir())
OUT=ROOT/'results'
seed=23
files={
 'Six healthy legs':'failure_leg_intact_neighbor_inhibit_seed23_2026-11-04.json',
 'FL disabled, existing controller':'failure_leg_fl_neighbor_inhibit_seed23_2026-11-04.json',
 'FL disabled, local recovery rule':'failure_leg_fl_neighbor_recover_seed23_2026-11-04.json'}
for label,name in files.items():
 r=json.loads((OUT/name).read_text())['trace_50hz']
 plt.plot([x['t'] for x in r],[x['advance_m']*100 for x in r],label=label)
plt.axvline(3,linestyle='--',label='Failure at t=3 s')
plt.title('FNK0052 CAD30: forward travel after single-leg support loss (seed 23)')
plt.xlabel('Time (seconds)');plt.ylabel('Forward travel (cm)')
plt.grid(alpha=.25);plt.legend(loc='upper left',fontsize=8)
plt.tight_layout();plt.savefig(OUT/'single_leg_failure_forward_seed23_2026-11-04.png',dpi=160);plt.close()
healthy=json.loads((OUT/files['Six healthy legs']).read_text())['trace_50hz']
broken=json.loads((OUT/files['FL disabled, existing controller']).read_text())['trace_50hz']
series=[healthy,broken];legs=('fl','ml','bl','br','mr','fr')
y=[np.mean([x['forces_N'][c] for x in r if x['t']>=3]) for r in series for c in legs]
x=np.arange(6);width=.38
plt.bar(x-width/2,[np.mean([p['forces_N'][c] for p in healthy if p['t']>=3]) for c in legs],width,label='Intact')
plt.bar(x+width/2,[np.mean([p['forces_N'][c] for p in broken if p['t']>=3]) for c in legs],width,label='After FL failure')
plt.xticks(x,legs);plt.ylabel('Mean normal contact load (N), t=3-8 s')
plt.title('FNK0052 CAD30: reaction forces redistribute after support loss')
plt.legend();plt.grid(axis='y',alpha=.25);plt.tight_layout()
plt.savefig(OUT/'single_leg_failure_loads_seed23_2026-11-04.png',dpi=160);plt.close()