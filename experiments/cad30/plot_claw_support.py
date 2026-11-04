"""Chart actual MuJoCo contact forces and geometry, without synthetic data."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

ROOT=Path(__file__).resolve().parents[2]
A=json.loads((ROOT/'results/c1_mu1.1_step.json').read_text())
B=json.loads((ROOT/'results/c2_mu1.1_no.json').read_text())
C=json.loads((ROOT/'results/c2_mu1.1_step.json').read_text())
fig,ax=plt.subplots(figsize=(11,4.7))
for case,name,ls in [(A,'One claw + rear step','-'),(B,'Two claws, four-leg support','-.'),(C,'Two claws + rear step (recovery risk)',':')]:
 x=[r['t'] for r in case['traces_50hz'] if r['t']>=2]
 y=[r['support_margin_mm'] for r in case['traces_50hz'] if r['t']>=2]
 ax.plot(x,y,ls,label=name,linewidth=1.2)
ax.axhline(0,linestyle='--',linewidth=.9)
for x in [2.15,3.25,5.25,6.5]:ax.axvline(x,linestyle=':',linewidth=.5,alpha=.5)
ax.set(xlabel='Time after 6-leg stand, s',ylabel='Signed COM support margin, mm',title='Observed support geometry under model-based claw actions')
ax.legend(loc='upper right',fontsize=8);ax.grid(alpha=.2);fig.tight_layout()
out=ROOT/'results/optimized_claw_support_margin_2026-11-04.png'
fig.savefig(out,dpi=160);plt.close(fig);print(out)