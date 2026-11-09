"""Explicit perturbations of the unchanged v2 surrogate; not hardware calibration."""
import xml.etree.ElementTree as ET
from .linkage_model import create

PROFILES={"nominal":{},"mass-090":{"mass_scale":.9},
          "mass-110":{"mass_scale":1.1},"friction-035":{"friction":.35}}


def make_model(profile="nominal",*,timestep=.001):
    if profile not in PROFILES:raise ValueError("Unknown model profile")
    base=create(timestep=timestep)
    settings=PROFILES[profile]
    if not settings:return base
    root=ET.fromstring(base)
    # ElementTree drops comments; physics, source geometry and controls remain
    # identical except for these explicitly declared sensitivity parameters.
    for g in root.iter('geom'):
        if 'mass' in g.attrib:g.set('mass',format(float(g.get('mass'))*settings.get('mass_scale',1),'.12g'))
    if 'friction' in settings:root.find('./default/geom').set('friction',f"{settings['friction']} .005 .0001")
    ET.indent(root,space='  ')
    return ET.tostring(root,encoding='unicode')+'\n'
