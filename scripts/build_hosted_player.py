#!/usr/bin/env python3
"""Export the existing Android browser player as a private, static Sites build.

Simulation and device-local updating are deliberately outside the hosted app.
Meshes retain their pinned upstream URLs and attribution. No CAD binaries are
copied. Three.js is copied from a separately integrity-verified vendor cache.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
THREE_URL = 'https://cdn.jsdelivr.net/npm/three@0.169.0/'


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_catalog(root: Path) -> dict:
    catalog = json.loads((root / 'hexapod/library.json').read_text())
    if catalog.get('schema') != 'hexapod-library/v1' or not catalog.get('entries'):
        raise ValueError('Unsupported or empty catalog')
    ids = set()
    for entry in catalog['entries']:
        if entry['id'] in ids or not re.fullmatch(r'/hexapod/(?:[a-z0-9-]+/)?', entry['href']):
            raise ValueError('Duplicate entry or invalid page: ' + entry['id'])
        ids.add(entry['id'])
        page = root / entry['href'].lstrip('/') / 'index.html'
        if not page.is_file():
            raise ValueError('Missing entry page: ' + entry['id'])
        for item in entry.get('files', []):
            relative = Path(item['path'])
            if relative.is_absolute() or '..' in relative.parts or not relative.parts or relative.parts[0] != 'hexapod':
                raise ValueError('Unsafe catalog path')
            path = root / relative
            if not path.is_file() or sha256(path) != item['sha256'] or path.stat().st_size != item['size']:
                raise ValueError('Catalog integrity mismatch: ' + item['path'])
    return catalog


def adapt_shell(text: str, build_id: str, replacement: str) -> str:
    first = text.index("  refresh.addEventListener('click',async()=>{")
    last = text.index("  play.addEventListener('click'", first)
    text = text[:first] + '  const HOSTED_BUILD_ID = ' + json.dumps(build_id) + ';\n' + replacement + '\n' + text[last:]
    text = text.replace("['/api/library','/hexapod/library.json']", "['/hexapod/library.json']")
    text = text.replace('async function readLibrary(){', 'async function readLibrary(signal){')
    text = text.replace("fetch(url,{cache:'no-store'})", "fetch(url,{cache:'no-store',signal})")
    text = text.replace('Обновить библиотеку', 'Проверить обновления')
    marker = "  const q=(s)=>shell.querySelector(s);"
    navigation = """
  const videos = document.createElement('a');
  videos.href = '/videos/';
  videos.className = 'hx-videos-link';
  videos.textContent = '▶ Видеозаписи';
  shell.querySelector('.hx-sheet-foot').prepend(videos);
"""
    if text.count(marker) != 1:
        raise ValueError('Unknown player shell structure')
    return text.replace(marker, navigation + marker)


def build(source: Path, site: Path, vendor: Path, media: Path | None = None) -> dict:
    source, site, vendor = source.resolve(), site.resolve(), vendor.resolve()
    if site == source or source in site.parents:
        raise ValueError('Site must be outside the source checkout')
    catalog = validate_catalog(source / 'docs')
    commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    inputs = {'source_commit':commit, 'exporter_sha256':sha256(Path(__file__)),
              'hosting':{str(p.relative_to(source)):sha256(p) for p in sorted((source/'hosting').rglob('*')) if p.is_file()},
              'vendor_manifest':sha256(vendor/'VENDOR_SHA256.json'),
              'media_manifest':sha256(media/'media.json') if media else None}
    build_id = hashlib.sha256(json.dumps(inputs,sort_keys=True).encode()).hexdigest()
    required = ['build/three.module.js', 'examples/jsm/controls/OrbitControls.js', 'examples/jsm/loaders/STLLoader.js', 'LICENSE']
    expected = json.loads((vendor / 'VENDOR_SHA256.json').read_text())
    for relative in required:
        path = vendor / 'vendor/three' / relative
        if not path.is_file() or sha256(path) != expected['vendor/three/' + relative]:
            raise ValueError('Vendor integrity mismatch: ' + relative)
    site.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.player-build-', dir=site))
    try:
        shutil.copytree(source / 'docs', stage, dirs_exist_ok=True)
        shutil.copytree(vendor / 'vendor', stage / 'vendor')
        for html in stage.rglob('*.html'):
            text = html.read_text().replace(THREE_URL, '/vendor/three/')
            html.write_text(text.replace('</head>', '<script src="/webgl-fallback.js"></script></head>'))
        shutil.copyfile(source / 'hosting/webgl-fallback.js', stage / 'webgl-fallback.js')
        shell = stage / 'hexapod/player-shell.js'
        shell.write_text(adapt_shell(shell.read_text(), build_id, (source / 'hosting/player-refresh.js').read_text()))
        css = stage / 'hexapod/player-shell.css'
        css.write_text(css.read_text() + '\n.hx-videos-link{display:block;text-align:center;color:#a8dfff;text-decoration:none;padding:13px;border:1px solid #33444f;border-radius:10px;margin-bottom:10px;font:600 14px system-ui}.hx-videos-link:focus-visible{outline:2px solid #a8dfff;outline-offset:3px}\n')
        shutil.copytree(source / 'hosting/videos', stage / 'videos')
        if media:
            metadata = json.loads((media / 'media.json').read_text())
            for item in metadata['videos']:
                path = media / item['filename']
                if Path(item['filename']).name != item['filename'] or sha256(path) != item['sha256']:
                    raise ValueError('Video integrity mismatch')
                shutil.copyfile(path, stage / 'videos' / item['filename'])
            shutil.copyfile(media / 'media.json', stage / 'videos/media.json')
        else:
            (stage / 'videos/media.json').write_text('{"videos":[]}\n')
        # The hosted copy contains rewritten HTML; publish its own exact digests.
        for entry in catalog['entries']:
            for item in entry.get('files', []):
                path = stage / item['path']
                item['sha256'], item['size'] = sha256(path), path.stat().st_size
        (stage / 'hexapod/library.json').write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
        report = {'schema':'hexapod-hosted-player/v1', 'source_commit':commit, 'build_id':build_id,
                  'catalog_revision':catalog['revision'], 'entries':len(catalog['entries']),
                  'video_count':len(metadata['videos']) if media else 0,
                  'mesh_policy':'pinned-upstream-URLs-no-hosted-CAD-copies'}
        (stage / 'hosted-version.json').write_text(json.dumps(report, indent=2) + '\n')
        validate_catalog(stage)
        if '/api/' in shell.read_text() or 'Termux' in shell.read_text():
            raise ValueError('Hosted player still depends on the local server')
        if (site / 'dist').exists():
            shutil.rmtree(site / 'dist')
        stage.rename(site / 'dist')
        return report
    finally:
        if stage.exists():
            shutil.rmtree(stage)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT)
    parser.add_argument('--site', type=Path, required=True)
    parser.add_argument('--vendor', type=Path, required=True)
    parser.add_argument('--media', type=Path)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.site, args.vendor, args.media), ensure_ascii=False))
