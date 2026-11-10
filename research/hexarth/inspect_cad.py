import json
from pathlib import Path
import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location

import argparse
ap=argparse.ArgumentParser();ap.add_argument('--step',required=True);ap.add_argument('--output',required=True);args=ap.parse_args()
doc=TDocStd_Document(TCollection_ExtendedString('HexArth'))
r=STEPCAFControl_Reader()
r.SetNameMode(True)
assert int(r.ReadFile(args.step))==1
assert r.Transfer(doc)
tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots=TDF_LabelSequence();tool.GetFreeShapes(roots)
rows=[]
def name(label):
    attr=TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),attr) else '?'
def walk(label,path,parent,depth):
    ref=TDF_Label()
    reference=tool.IsReference_s(label)
    target=label
    loc=parent
    if reference:
        tool.GetReferredShape_s(label,ref);target=ref
        loc=parent.Multiplied(tool.GetLocation_s(label))
    nm=name(target)
    this=path+'/'+nm
    children=TDF_LabelSequence();tool.GetComponents_s(target,children,False)
    row={'index':len(rows),'path':this,'depth':depth,'children':children.Length(),'name':nm}
    tr=loc.Transformation()
    row['transform']=[[tr.Value(i,j) for j in range(1,5)] for i in range(1,4)]
    if not children.Length():
        raw=tool.GetShape_s(target)
        shape=cq.Shape.cast(raw).moved(cq.Location(loc))
        try:
            bb=shape.BoundingBox(); c=shape.Center()
        except Exception:
            return
        row.update(center=[c.x,c.y,c.z],bounds=[bb.xmin,bb.ymin,bb.zmin,bb.xmax,bb.ymax,bb.zmax],volume=shape.Volume(),solids=len(shape.Solids()))
    rows.append(row)
    if depth==0: print('Top-level assembly',nm,'components',children.Length(),flush=True)
    for j in range(1,children.Length()+1): walk(children.Value(j),this+f'[{j}]',loc,depth+1)
for i in range(1,roots.Length()+1):walk(roots.Value(i),'',TopLoc_Location(),0)
Path(args.output).write_text(json.dumps(rows,indent=2,ensure_ascii=False))
print('TOTAL',len(rows))
