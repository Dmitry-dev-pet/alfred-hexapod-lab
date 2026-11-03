"""Plot actual MuJoCo normal force traces, threshold and observed liftoffs."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'results/load_feedback_neighbor_force_gate_supported_seed11_nominal_mu0p6.json').read_text())
rows=r['force_trace_50hz'];ts=np.array([x['t'] for x in rows]);
fig,ax=plt.subplots(figsize=(12,5.5),layout='constrained')
for leg in ('fl','fr','ml'):
 ax.plot(ts,np.array([x['forces_N'][leg] for x in rows]),label=f'{leg}: нормальная сила')
ax.axhline(r['max_self_load_for_lift_N'],linewidth=1.4,linestyle='--',label='Порог подъёма ноги (сглаженный сигнал)')
ls=[e['t'] for e in r['events'] if e['event']=='liftoff' and e['leg']=='fl']
for i,t in enumerate(ls):ax.axvline(t,alpha=.2,linewidth=1.0,label='Разрешённый подъём fl' if i==0 else None)
ax.set_xlabel('Время движения, с');ax.set_ylabel('Сила опоры, Н')
ax.set_title('CAD30 / μ=0,6: реальные силы контактов из MuJoCo, старт №11')
ax.set_xlim(0,8);ax.set_ylim(bottom=0)
ax.legend(ncols=2,fontsize=9);ax.grid(alpha=.2)
out=ROOT/'results/load_feedback_seed11_mu06_forces.png'
fig.savefig(out,dpi=150)
print(out,out.stat().st_size)
