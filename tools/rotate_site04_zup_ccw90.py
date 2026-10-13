#!/usr/bin/env python3
"""Rotate an existing Z-up PCD/VGH set +90 degrees around map-origin +Z."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from compact_vgh_trajectory import read_graph


def rotate_xy(values):
    result = values.copy()
    result[..., 0] = -values[..., 1]
    result[..., 1] = values[..., 0]
    return result


def convert(source, output):
    names = ['prior.pcd', 'site_04.vgh', 'site_04_compact_trajectory.vgh']
    inputs = {name: (source / name).read_bytes() for name in names}
    raw = inputs['prior.pcd']
    offset = raw.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = raw[:offset].decode()
    meta = {line.split()[0]: line.split()[1:] for line in header.splitlines()
            if line and not line.startswith('#')}
    if set(meta['TYPE']) != {'F'} or set(meta['SIZE']) != {'4'} or set(meta['COUNT']) != {'1'}:
        raise ValueError('Expected binary PCD with scalar float32 fields')
    fields = meta['FIELDS']
    points = np.frombuffer(raw[offset:], dtype='<f4').reshape(-1, len(fields)).copy()
    if len(points) != int(meta['POINTS'][0]):
        raise ValueError('PCD point count mismatch')
    for x, y in [('x', 'y'), ('normal_x', 'normal_y')]:
        if x in fields and y in fields:
            columns = [fields.index(x), fields.index(y)]
            points[:, columns] = rotate_xy(points[:, columns])
    tx, ty, tz, w, x, y, z = map(float, meta['VIEWPOINT'])
    h = math.sqrt(0.5)
    viewpoint = [-ty, tx, tz, h*(w-z), h*(x-y), h*(x+y), h*(w+z)]
    header = '\n'.join(
        'VIEWPOINT ' + ' '.join(f'{v:.17g}' for v in viewpoint)
        if line.startswith('VIEWPOINT ') else line for line in header.splitlines()) + '\n'

    outputs = {'prior.pcd': header.encode() + points.tobytes()}
    node_counts = {}
    for name in names[1:]:
        before = read_graph(source / name)
        lines = []
        for line in inputs[name].decode().splitlines():
            if not line.strip():
                continue
            tokens = line.split()
            original = tokens.copy()
            # Position and both surface vectors. The pillar sentinel (0,0,-1)
            # stays unchanged under this yaw rotation, as do all Z components.
            for start in (2, 5, 8):
                a, b = float(tokens[start]), float(tokens[start+1])
                tokens[start], tokens[start+1] = f'{-b:.6f}', f'{a:.6f}'
            assert tokens[:2] == original[:2] and tokens[11:] == original[11:]
            lines.append(' '.join(tokens) + ' ')
        outputs[name] = ('\n'.join(lines) + '\n').encode()
        node_counts[name] = len(before)

    # Never replace an existing asset directory, even on a repeated invocation.
    output.mkdir(parents=True, exist_ok=False)
    for name, data in outputs.items():
        (output / name).write_bytes(data)
    for name in names[1:]:
        before, after = read_graph(source / name), read_graph(output / name)
        assert before.keys() == after.keys()
        for ident, node in before.items():
            assert node['edges'] == after[ident]['edges']
            np.testing.assert_allclose(after[ident]['position'], rotate_xy(np.array(node['position'])), atol=1e-6)
    report = {
        'source_directory': str(source.resolve()), 'output_directory': str(output.resolve()),
        'rotation': [[0,-1,0], [1,0,0], [0,0,1]], 'determinant': 1,
        'yaw_offset_deg': 90, 'pivot_in_source_map': [0,0,0],
        'coordinate_rule': 'x_new=-y_old; y_new=x_old; z_new=z_old; yaw_new=wrap(yaw_old+pi/2)',
        'points': len(points), 'graph_nodes': node_counts, 'topology_unchanged': True,
        'source': {name: hashlib.sha256(data).hexdigest() for name, data in inputs.items()},
        'output': {name: hashlib.sha256(data).hexdigest() for name, data in outputs.items()},
        'field_alignment_verified': False,
    }
    (output / 'conversion.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    convert(args.source, args.output)
