"""Diagnostic raster of *observed* leg swing states (not expected motor phases)."""
from pathlib import Path
import json
import matplotlib.pyplot as plt
import numpy as np

ROOT=next((p for p in Path(__file__).resolve().parents if (p/'decentralized_ablation.py').exists()), None)
if ROOT is None: raise RuntimeError('Missing decentralized_ablation.py in experiment folder')
MODES=[('independent','Без сообщений'),('neighbor_inhibit','Запрет соседнего переноса'),('neighbor_touchdown','Запрет + сигнал касания')]
NAMES=('fl','ml','bl','br','mr','fr')
SEED=23
fig,ax=plt.subplots(figsize=(13.0,8.1),layout='constrained')
labels=[];yidx=[]
for n,(mode,label) in enumerate(MODES):
    r=json.loads((ROOT/'results'/f'decentral_{mode}_seed{SEED}_nominal_mu1p1.json').read_text())
    time=np.asarray([e['t'] for e in r['trace_50hz']]);interval=0.02
    for j,leg in enumerate(NAMES):
        y=(len(MODES)-1-n)*7+j
        labels.append(leg); yidx.append(y)
        for e in r['trace_50hz']:
            if leg in e['swinging']:
                ax.broken_barh([(e['t'],interval)],(y-.34,.68),facecolors=f'C{n}',linewidth=0)
    ax.text(8.07,(len(MODES)-1-n)*7+2.4,label,fontsize=11,ha='left',va='center',fontweight='bold')
ax.set_xlim(0,10.8)
ax.set_ylim(-1,21)
ax.set_yticks(yidx,labels)
ax.set_xticks(np.arange(0,9,1))
ax.set_xlabel('Время физической симуляции, с')
ax.set_ylabel('CAD-код ноги (не нумерация Freenove)')
ax.set_title('Переносы ног из MuJoCo: одинаковый случайный старт №23, без заданной общей фазы')
ax.grid(axis='x',alpha=.22)
ax.axhline(6.5,alpha=.3)
ax.axhline(13.5,alpha=.3)
# Colored bands show controller-declared swing; contact count is a separate observed signal.
out=ROOT/'results'/'decentralized_seed23_liftoff_raster.png'
fig.savefig(out,dpi=145)
print(out, out.stat().st_size)