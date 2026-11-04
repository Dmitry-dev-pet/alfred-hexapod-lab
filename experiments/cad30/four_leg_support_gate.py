"""Diagnostic oracle for whether a selected support could be safely released.

IMPORTANT: this uses MuJoCo's *true* COM and contact coordinates.  It is only
an analysis reference for the real contact/force/IMU estimator, NOT a controller
that can run unmodified on a physical FNK0052.
"""
from __future__ import annotations
import numpy as np
from front_pair_trick import SUPPORT, FRONT, contact_points, margin

def diagnostic_liftoff_gate(model, data, min_margin_m:float=0.005):
    if min_margin_m < 0: raise ValueError('margin must be nonnegative')
    cp=contact_points(model,data)
    root=int(model.body('base_link_2').id)
    com_xy=np.asarray(data.subtree_com[root][:2],dtype=float)
    result={}
    for c in SUPPORT:
        neighbors=[o for o in SUPPORT if o!=c]
        valid=all(cp[k] for k in neighbors)
        geom_margin=margin(com_xy,[p for k in neighbors for p in cp[k]]) if valid else None
        result[c]={'prospective_supports':neighbors,'com_margin_m':None if geom_margin is None else float(geom_margin),
                   'passes_static_gate':bool(valid and geom_margin is not None and geom_margin>=min_margin_m),
                   'static_test_only':True}
    return result
