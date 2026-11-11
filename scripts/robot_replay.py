"""Data-only Robot Replay v1 writer; independent of robot and simulator."""
import base64
import gzip
import hashlib
import json
import pathlib
import struct


def block(values, dtype='f64'):
    codes = {'f64': 'd', 'f32': 'f', 'u32': 'I'}
    raw = struct.pack('<' + str(len(values)) + codes[dtype], *values)
    return encoded_block(raw, dtype, len(values))


def encoded_block(raw, dtype, count):
    return {'encoding': dtype+'le-gzip-base64', 'count': count,
            'sha256': hashlib.sha256(raw).hexdigest(),
            'data': base64.b64encode(gzip.compress(raw, compresslevel=6, mtime=0)).decode('ascii')}


def write_package(doc, output):
    assert doc['schema'] == 'robot-replay/v1'
    raw = json.dumps(doc, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()
    payload = gzip.compress(raw, compresslevel=6, mtime=0)
    path = pathlib.Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    return {'schema': doc['schema'], 'file': path.name, 'size_bytes': len(payload),
            'sha256': hashlib.sha256(payload).hexdigest(), 'json_bytes': len(raw),
            'nodes': len(doc['nodes']), 'meshes': len(doc['meshes']),
            'clips': [{'id': c['id'], 'frames': c['frame_count'], 'duration_s': c['duration_s']} for c in doc['clips']]}


def main():
    import argparse
    p = argparse.ArgumentParser(description='Compress a complete Robot Replay v1 JSON into one .robotreplay file')
    p.add_argument('scene_json', type=pathlib.Path)
    p.add_argument('output', type=pathlib.Path)
    args = p.parse_args()
    print(json.dumps(write_package(json.loads(args.scene_json.read_text()), args.output)))


if __name__ == '__main__':
    main()
