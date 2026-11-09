from pathlib import Path
import json
import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence,TDF_Label
from OCP.TopLoc import TopLoc_Location
from cadquery.occ_impl.importers.assembly import _get_name,_get_ref_color,_get_shape_color

def read_parts(path):
    reader=STEPCAFControl_Reader();reader.SetColorMode(True);reader.SetNameMode(True)
    assert reader.ReadFile(str(path))==IFSelect_RetDone
    doc=TDocStd_Document(TCollection_ExtendedString('XmXCAF'));assert reader.Transfer(doc)
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());colors=XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    roots=TDF_LabelSequence();tool.GetFreeShapes(roots);parts=[]
    def visit(label,placement,path):
        loc=placement.Multiplied(tool.GetLocation_s(label))
        name=_get_name(label);ref=label
        if tool.IsReference_s(label):
            ref=TDF_Label();assert tool.GetReferredShape_s(label,ref)
        if tool.IsAssembly_s(ref):
            children=TDF_LabelSequence();tool.GetComponents_s(ref,children)
            for n in range(1,children.Length()+1):visit(children.Value(n),loc,path+[f'{name}[{n}]'])
        else:
            raw=tool.GetShape_s(ref)
            if raw.IsNull():return
            shape=cq.Shape.cast(raw).moved(cq.Location(loc))
            if not shape.Faces():return
            color=_get_ref_color(label) or _get_shape_color(raw,colors)
            b=shape.BoundingBox()
            parts.append({'id':len(parts),'name':name,'reference_name':_get_name(ref),'path':path+[name],
                'color':list(color.toTuple()) if color else None,'solids':len(shape.Solids()),
                'bounds':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax],
                'center':[(b.xmin+b.xmax)/2,(b.ymin+b.ymax)/2,(b.zmin+b.zmax)/2],'shape':shape})
    for n in range(1,roots.Length()+1):visit(roots.Value(n),TopLoc_Location(),[])
    return parts

if __name__=='__main__':
    parts=read_parts(Path(__file__).parent/'source/OriginalRobotModel.stp')
    data=[{k:v for k,v in p.items() if k!='shape'} for p in parts]
    Path(__file__).with_name('assembly-inspection.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
    print(json.dumps(data,ensure_ascii=False,indent=2))
