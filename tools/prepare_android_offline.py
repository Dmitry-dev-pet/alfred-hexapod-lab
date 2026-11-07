#!/usr/bin/env python3
"""Build a self-contained, local-only Hexapod Lab viewer (no Python dependencies).

Run from a pinned hexapod-lab checkout after review. All downloaded files are
pinned by version / original Git SHA and verified before use; no uploads occur.
"""
import argparse
import base64
import hashlib
import io
import json
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.parse
import urllib.request
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

THREE_VERSION = '0.169.0'
THREE_METADATA = f'https://registry.npmjs.org/three/{THREE_VERSION}'
THREE_TARBALL = f'https://registry.npmjs.org/three/-/three-{THREE_VERSION}.tgz'
CAD_REV = '5c9a84c16335b30036e451c691d2c544abf4bc72'
REPLAYS = ('cad-rig-60s.json', 'cad-rig-claws-10s.json', 'cad-rig-six-to-four-7s.json')
PAGES = ('hexapod/index.html', 'hexapod/claws/index.html', 'hexapod/six-to-four/index.html')
THREE_FILES = (
    'build/three.module.js',
    'examples/jsm/controls/OrbitControls.js',
    'examples/jsm/loaders/STLLoader.js',
    'LICENSE',
)
OLD_THREE = f'https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/build/three.module.js'
OLD_ADDONS = f'https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/examples/jsm/'
NEW_THREE = '/vendor/three/build/three.module.js'
NEW_ADDONS = '/vendor/three/examples/jsm/'


def get_bytes(url: str, limit: int) -> bytes:
    req = urllib.request.Request(url, headers={'User-Agent': 'AlfredHexapodOfflineInstaller/1.0'})
    with urllib.request.urlopen(req, timeout=35) as response:
        if response.status != 200:
            raise RuntimeError(f'HTTP {response.status}: {url}')
        body = response.read(limit + 1)
        if len(body) > limit:
            raise RuntimeError(f'Exceeded download bound: {url}')
        return body


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def install_three(dest: Path, archive: Path | None) -> dict:
    if archive is None:
        md = json.loads(get_bytes(THREE_METADATA, 200_000))
        if md.get('name') != 'three' or md.get('version') != THREE_VERSION:
            raise RuntimeError('npm metadata identity mismatch')
        integrity = md.get('dist', {}).get('integrity', '')
        if not integrity.startswith('sha512-'):
            raise RuntimeError('npm sha512 integrity not provided')
        expected = base64.b64decode(integrity[7:], validate=True)
        data = get_bytes(THREE_TARBALL, 20_000_000)
        if hashlib.sha512(data).digest() != expected:
            raise RuntimeError('three npm integrity mismatch')
    else:
        data = archive.read_bytes()
    records = {}
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as tar:
        for relative in THREE_FILES:
            source = tar.extractfile('package/' + relative)
            if source is None:
                raise RuntimeError('Missing three: ' + relative)
            payload = source.read(2_500_001)
            if not payload or len(payload) > 2_500_000:
                raise RuntimeError('Unexpected three size: ' + relative)
            path = dest / 'vendor' / 'three' / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            records['vendor/three/' + relative] = digest(payload)
    for path in THREE_FILES[:3]:
        script = (dest / 'vendor' / 'three' / path).read_text()
        if re.search(r'from\s+[\'\"](?:https?://|/)', script):
            raise RuntimeError('Three.js tries to import a remote module')
    if shutil.which('node'):
        for relative in THREE_FILES[:3]:
            # node --check works for ES module syntax regardless of file extension.
            subprocess.run(['node', '--check', str(dest / 'vendor' / 'three' / relative)],
                           check=True, capture_output=True, timeout=20)
    return records


def cad_bytes(name: str, url: str, expected_sha: str, asset_zip: Path | None) -> tuple[str, bytes]:
    if asset_zip:
        with zipfile.ZipFile(asset_zip) as z:
            data = z.read('meshes/' + name)
    else:
        data = get_bytes(url, 2_000_000)
    if digest(data) != expected_sha:
        raise RuntimeError('CAD SHA256 mismatch: ' + name)
    return name, data


def make_bundle(source: Path, target: Path, manifest_path: Path,
                three_archive: Path | None, cad_archive: Path | None) -> dict:
    source_docs = source / 'docs'
    if not source_docs.is_dir():
        raise RuntimeError('Not a Hexapod Lab source checkout (docs missing)')
    if target.exists():
        raise RuntimeError('Target exists: refusing to overwrite ' + str(target))
    checksums = json.loads(manifest_path.read_text())
    if checksums['source_commit'] != CAD_REV or len(checksums['files']) != 30:
        raise RuntimeError('Unexpected CAD verification manifest')
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix='.hexapod-offline-stage-', dir=target.parent))
    try:
        shutil.copytree(source_docs, tmp, dirs_exist_ok=True)
        cad_sets = []
        all_urls = {}
        for name in REPLAYS:
            f = tmp / 'hexapod' / 'data' / name
            rig = json.loads(f.read_text())
            meshes = rig.get('meshes', [])
            filenames = [m['filename'] for m in meshes]
            if len(meshes) != 30 or len(set(filenames)) != 30 or set(filenames) != set(checksums['files']):
                raise RuntimeError('CAD list mismatch: ' + name)
            if not rig.get('frames') or not rig.get('bodies'):
                raise RuntimeError('No physical replay in ' + name)
            for mesh in meshes:
                filename = mesh['filename']
                url = mesh['url']
                parts = urllib.parse.urlparse(url)
                expected_path = (f'/MattiaGrigoli/autonomous_hexapod/{CAD_REV}/'
                                 f'src/freenove_description/meshes/{filename}')
                if parts.scheme != 'https' or parts.netloc != 'raw.githubusercontent.com' or parts.path != expected_path:
                    raise RuntimeError('Unexpected upstream asset location: ' + filename)
                all_urls[filename] = url
                mesh['url'] = '/assets/stl/' + filename
            f.write_text(json.dumps(rig, separators=(',', ':'), ensure_ascii=False))
            cad_sets.append({'replay': name, 'frames': len(rig['frames']), 'seconds': rig.get('duration_s')})
        if len(all_urls) != 30:
            raise RuntimeError('CAD source set incomplete')
        outdir = tmp / 'assets' / 'stl'
        outdir.mkdir(parents=True)
        with ThreadPoolExecutor(max_workers=5) as pool:
            results = list(pool.map(lambda x: cad_bytes(x, all_urls[x], checksums['files'][x], cad_archive),
                                    sorted(all_urls)))
        for name, data in results:
            (outdir / name).write_bytes(data)
        vendor = install_three(tmp, three_archive)
        for name in PAGES:
            file = tmp / name
            html = file.read_text()
            if html.count(OLD_THREE) != 1 or html.count(OLD_ADDONS) != 1:
                raise RuntimeError('Unknown importmap: ' + name)
            html = html.replace(OLD_THREE, NEW_THREE).replace(OLD_ADDONS, NEW_ADDONS)
            file.write_text(html)
        for script in ('hexapod/app.js', 'hexapod/claws/app.js', 'hexapod/six-to-four/app.js'):
            js = (tmp / script).read_text()
            if re.search(r'https?://[^\s\'\"]+\.(?:js|json|stl)', js):
                raise RuntimeError('Potential hard-coded remote asset in ' + script)
        shutil.copyfile(Path(__file__).with_name('local_viewer_server.py'), tmp / 'local_viewer_server.py')
        info = {'format': 'hexapod-offline/v1', 'three_version': THREE_VERSION,
                'source_head': subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip(),
                'source_cad_commit': CAD_REV, 'replays': cad_sets,
                'files_sha256': {**{'assets/stl/' + n: checksums['files'][n] for n in sorted(all_urls)}, **vendor}}
        (tmp / 'OFFLINE_MANIFEST.json').write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n')
        (tmp / 'START_HERE.txt').write_text(
            'Hexapod Lab – offline local replay for Android Termux.\n'
            'Run: python local_viewer_server.py start\n'
            'Stop: python local_viewer_server.py stop\n'
            'Status: python local_viewer_server.py status\n'
            'Open http://127.0.0.1:8808/hexapod/\n'
            'More: http://127.0.0.1:8808/hexapod/claws/ and /hexapod/six-to-four/\n'
            'Viewer is a replay, not live MuJoCo. Original CAD © Freenove; see upstream third-party notice.\n')
        # All assets and replay data must be on disk before the atomic rename.
        for needed in ('vendor/three/build/three.module.js', 'hexapod/data/cad-rig-60s.json',
                       'hexapod/data/cad-rig-claws-10s.json', 'hexapod/data/cad-rig-six-to-four-7s.json'):
            if not (tmp / needed).is_file():
                raise RuntimeError('Missing offline file: ' + needed)
        tmp.rename(target)
        return info
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='.', type=Path)
    parser.add_argument('--dest', default=Path.home() / 'hexapod-viewer-local', type=Path)
    parser.add_argument('--three-tarball', type=Path, help='Optional verified cached npm package for offline builds')
    parser.add_argument('--cad-zip', type=Path, help='Optional local pinned original CAD archive (private, not republished)')
    args = parser.parse_args()
    source = args.source.expanduser().resolve()
    target = args.dest.expanduser().resolve()
    if target == source or source in target.parents:
        raise SystemExit('Target must be outside the source checkout')
    info = make_bundle(source, target, Path(__file__).with_name('stl_sha256.json'),
                       args.three_tarball, args.cad_zip)
    print(json.dumps({'status': 'BUILT', 'source_head': info['source_head'],
                      'replays': info['replays'], 'destination': str(target)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
