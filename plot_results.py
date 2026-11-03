"""Optional matplotlib visualization. Needs matplotlib, not needed for core tests."""
import json
from pathlib import Path
import matplotlib.pyplot as plt

cases = json.loads(Path('results/pilot_results.json').read_text())
fig, ax = plt.subplots(figsize=(9.5, 4.5))
for case in cases:
    t=[p['t'] for p in case['trace']]
    y=[p['margin_m']*1000 if p['margin_m'] is not None else -30 for p in case['trace']]
    label=case['strategy'] + (' / disabled L0 at 6 s' if case['failed_leg'] is not None else ' / all legs')
    ax.plot(t,y,label=label)
ax.axhline(0,linewidth=1,linestyle='--')
ax.axvline(6,linewidth=1,linestyle=':',alpha=0.55)
ax.set_xlabel('Simulated time (s)')
ax.set_ylabel('Static support margin (mm)')
ax.set_title('FNK0052 pilot: geometric stability only (not physics simulation)')
ax.legend(loc='upper right',fontsize=8)
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig('results/pilot_margins.png',dpi=160)