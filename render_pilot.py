"""Render pilot plots with Pillow (optional); independent of matplotlib."""
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

cases=json.loads(Path('results/pilot_results.json').read_text())
W,H=1100,560
im=Image.new('RGB',(W,H),(252,253,254)); d=ImageDraw.Draw(im)
left,top,right,bottom=90,45,1040,440
colors=[(23,117,166),(21,154,112),(226,103,42),(108,66,169)]
for i in range(6):
    v=-40+i*50
    y=bottom-(v+40)/250*(bottom-top)
    d.line((left,y,right,y),fill=(218,226,233),width=1)
    d.text((25,y-7),f'{v:>3} mm',fill=(49,62,75))
for s in range(0,13,2):
    x=left+s/12*(right-left)
    d.line((x,top,x,bottom),fill=(234,238,242),width=1)
    d.text((x-8,bottom+10),str(s),fill=(49,62,75))
def px(t):return left+t/12*(right-left)
def py(margin_mm):return bottom-((margin_mm+40)/250)*(bottom-top)
d.line((left,py(0),right,py(0)),fill=(100,108,116),width=2)
d.line((px(6),top,px(6),bottom),fill=(160,155,149),width=2)
for case,color in zip(cases,colors):
    pts=[(px(p['t']), py(p['margin_m']*1000 if p['margin_m'] is not None else -40)) for p in case['trace']]
    d.line(pts,fill=color,width=3,joint='curve')
d.text((left,10),'Freenove pilot: geometric support margin (NOT dynamics)',fill=(25,34,45))
d.text((450,bottom+30),'Model time (seconds)',fill=(55,65,80))
labels=['Tripod / intact','Local rules / intact','Tripod / leg 0 disabled at 6 s','Local rules / leg 0 disabled at 6 s']
for i,(col,label) in enumerate(zip(colors,labels)):
    x=85+(i%2)*510; y=492+(i//2)*26
    d.line((x,y+6,x+28,y+6),fill=col,width=4)
    d.text((x+37,y),label,fill=(34,40,45))
im.save('results/pilot_margins.png')