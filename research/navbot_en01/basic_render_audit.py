"""Check the original CAD surfaces between saved poses using player interpolation."""
import argparse, json, math
from pathlib import Path
from .contact_mesh_audit import MeshAudit


def check(dataset, mesh):
    import numpy as np
    dataset = Path(dataset)
    audit = MeshAudit(mesh, (dataset/'model.xml').read_text())
    results = []
    for path in sorted((dataset/'raw').glob('*_raw.json')):
        record = json.loads(path.read_bytes())
        count, failures = 0, []
        for first, second in zip(record['frames'], record['frames'][1:]):
            a, b = np.array(first['body_poses']), np.array(second['body_poses'])
            for k in (.25, .5, .75):
                out = np.empty_like(a)
                out[:, :3] = (1-k)*a[:, :3]+k*b[:, :3]
                for j in range(9):
                    p, q = a[j, 3:], b[j, 3:]
                    dot = float(p@q)
                    if dot < 0:
                        q, dot = -q, -dot
                    if dot > .9995:
                        v = (1-k)*p+k*q
                        v /= np.linalg.norm(v)
                    else:
                        theta = math.acos(min(1., dot))
                        v = (math.sin((1-k)*theta)*p+math.sin(k*theta)*q)/math.sin(theta)
                    out[j, 3:] = v
                hits = audit.intersections(out.tolist())
                if hits:
                    failures.append({'t':(1-k)*first['t']+k*second['t'], 'hits':hits})
                count += 1
        results.append({'case':record['summary']['case'], 'interpolated_poses':count, 'unexpected_intersections':failures})
        print(path.stem, 'interpolation failures', len(failures), flush=True)
    return {'schema':'en01-basic-render-surface-audit/v1', 'fractions':[.25,.5,.75], 'rows':results,
            'scope':'Original 73-part surfaces, intended joint mates excluded; sampled interpolation, not a continuous solid guarantee'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--cad-mesh', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    result = check(a.dataset, a.cad_mesh)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(result, indent=2)+'\n')
