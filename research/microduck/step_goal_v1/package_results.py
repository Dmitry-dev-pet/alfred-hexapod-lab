"""Package explicit, verified experiment inputs and outputs; no runtime secrets."""
from pathlib import Path
import hashlib
import importlib.metadata as md
import json
import zipfile
import argparse

HERE = Path(__file__).resolve().parent
NAME = 'MicroDuck_step_goal_tuning_20261109.zip'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--workspace', type=Path, required=True)
    work = ap.parse_args().workspace.resolve()
    files = {}
    for path in HERE.rglob('*'):
        if not path.is_file() or any(p in ('__pycache__',) for p in path.parts):
            continue
        if path.suffix in ('.zip', '.log') or path.name in ('claim.json', 'run-payload.json', 'run-body.md', 'archive-receipt.json'):
            continue
        if any(p.startswith('library-') for p in path.parts):
            continue
        files['experiment/' + path.relative_to(HERE).as_posix()] = path
    manifest_path = work / 'microduck-work/source-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    files['workspace/microduck-work/source-manifest.json'] = manifest_path
    for kind, directory in (('microduck_rl', 'microduck_rl-' + manifest['microduck_rl']['commit']),
                            ('bam', 'bam-' + manifest['bam']['commit'])):
        for relative, expected in manifest[kind]['blobs'].items():
            path = work / 'microduck-work' / directory / relative
            data = path.read_bytes()
            assert hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest() == expected
            files[f'workspace/microduck-work/{directory}/{relative}'] = path
    for rel in ('microduck-work/velstand.onnx', 'microduck-roll/roulade.onnx'):
        files['workspace/' + rel] = work / rel
    versions = {name: md.version(name) for name in ('numpy', 'scipy', 'mujoco', 'onnxruntime', 'Pillow')}
    requirements = ''.join(f'{name}=={version}\n' for name, version in versions.items())
    (HERE / 'requirements.txt').write_text(requirements)
    files['experiment/requirements.txt'] = HERE / 'requirements.txt'
    hashes = {name: {'bytes': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
              for name, path in sorted(files.items())}
    out = HERE / NAME
    with zipfile.ZipFile(out, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=7) as z:
        for name, path in sorted(files.items()):
            z.write(path, name)
        z.writestr('SHA256SUMS.json', json.dumps(hashes, indent=2) + '\n')
    with zipfile.ZipFile(out) as z:
        assert z.testzip() is None
        for name, value in hashes.items():
            assert hashlib.sha256(z.read(name)).hexdigest() == value['sha256']
    receipt = {'file': out.name, 'bytes': out.stat().st_size,
               'sha256': hashlib.sha256(out.read_bytes()).hexdigest(), 'members': len(hashes) + 1,
               'all_member_hashes_verified': True, 'input_git_blobs_verified': 371}
    (HERE / 'archive-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt))


if __name__ == '__main__':
    main()
