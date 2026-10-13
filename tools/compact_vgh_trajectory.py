#!/usr/bin/env python3
"""Contract redundant connected trajectory samples; retain every obstacle vertex.

Clusters are connected through existing trajectory edges, have identical semantic
flags, and stay within radius of an unchanged representative. Original files are
never overwritten. This preserves graph connectivity, not physical clearance.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path


def read_graph(path):
    nodes = {}
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split('|')
        assert len(parts) == 4, 'Expected four adjacency lists'
        head = parts[0].split()
        assert len(head) >= 15
        ident = int(head[0])
        assert ident not in nodes
        fields = head[:15]
        p = tuple(float(x) for x in fields[2:5])
        assert all(math.isfinite(x) for x in p)
        nodes[ident] = {'fields': fields, 'position': p,
                        'edges': [set(map(int, head[15:]))] + [set(map(int, s.split())) for s in parts[1:]]}
    assert nodes
    for node in nodes.values():
        assert all(edge <= nodes.keys() for edge in node['edges']), 'Dangling edge in source'
    return nodes


def signature(node):
    # Retain corner/boundary nodes even when coincident. Only ordinary trajectory
    # points with identical free type, surface directions and flags may merge.
    f = node['fields']
    if int(f[13]) != 1 or int(f[14]) != 0:
        return None
    return (int(f[1]), *map(float, f[5:11]), *map(int, f[11:15]))


def component_labels(nodes, index):
    indices = (index,) if isinstance(index, int) else index
    adjacency = {i: set().union(*(n['edges'][k] for k in indices)) for i, n in nodes.items()}
    for i, edges in list(adjacency.items()):
        for j in list(edges):
            adjacency[j].add(i)
    labels = {}
    for seed in nodes:
        if seed in labels:
            continue
        todo = [seed]
        labels[seed] = seed
        while todo:
            for j in adjacency[todo.pop()]:
                if j not in labels:
                    labels[j] = seed
                    todo.append(j)
    return labels


def compact(nodes, radius):
    mapping, clusters = {}, {}
    for seed, node in nodes.items():
        if seed in mapping:
            continue
        mapping[seed] = seed
        clusters[seed] = [seed]
        sig = signature(node)
        if sig is None:
            continue
        todo = deque([seed])
        while todo:
            current = todo.popleft()
            for neighbor in sorted(nodes[current]['edges'][3]):
                if neighbor in mapping or signature(nodes[neighbor]) != sig:
                    continue
                # Require a reciprocal existing trajectory edge, not proximity alone.
                if current not in nodes[neighbor]['edges'][3]:
                    continue
                if math.dist(node['position'], nodes[neighbor]['position']) <= radius:
                    mapping[neighbor] = seed
                    clusters[seed].append(neighbor)
                    todo.append(neighbor)
    output = {}
    for representative, members in clusters.items():
        node = nodes[representative]
        edges = [{mapping[j] for i in members for j in nodes[i]['edges'][k]} - {representative}
                 for k in range(4)]
        output[representative] = dict(node, edges=edges)
    return output, mapping, clusters


def main(args):
    source, dest = Path(args.input), Path(args.output)
    assert 0 <= args.radius <= .05, 'Maximum supported displacement is 5 cm'
    assert source.resolve() != dest.resolve() and not dest.exists()
    nodes = read_graph(source)
    output, mapping, clusters = compact(nodes, args.radius)
    max_shift = max(math.dist(n['position'], nodes[mapping[i]]['position']) for i, n in nodes.items())
    assert max_shift <= args.radius + 1e-12
    assert all(mapping[i] == i for i, n in nodes.items() if signature(n) is None)
    # FAR can reconnect existing trajectory edges (DynamicGraph::IsValidConnect),
    # even if currently absent from connect_nodes. Check their combined topology
    # and the trajectory topology separately; never join disconnected trajectories.
    components = {}
    for index, name in (((0, 3), 'navigation_and_trajectory'), (3, 'trajectory')):
        before, after = component_labels(nodes, index), component_labels(output, index)
        correspondence, reverse = {}, {}
        for i in nodes:
            a, b = before[i], after[mapping[i]]
            assert correspondence.setdefault(a, b) == b, 'Lost source component'
            assert reverse.setdefault(b, a) == a, 'Joined previously separate components'
        components[name] = len(correspondence)
    dest.parent.mkdir(parents=True, exist_ok=True)
    # Keep a final space: the upstream parser tokenizes only space-terminated tokens.
    text = '\n'.join(' '.join(n['fields']) + ' ' + ' | '.join(
        ' '.join(map(str, sorted(e))) for e in n['edges']) + ' ' for n in output.values()) + '\n'
    dest.write_text(text)
    restored = read_graph(dest)
    assert len(restored) == len(output)
    assert all(restored[i] == output[i] for i in output)
    report = {'source': str(source.resolve()), 'output': str(dest.resolve()),
              'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'output_sha256': hashlib.sha256(dest.read_bytes()).hexdigest(),
              'nodes_before': len(nodes), 'nodes_after': len(output), 'radius_m': args.radius,
              'max_representative_displacement_m': max_shift,
              'edges_before': [sum(len(n['edges'][k]) for n in nodes.values()) for k in range(4)],
              'edges_after': [sum(len(n['edges'][k]) for n in output.values()) for k in range(4)],
              'components_preserved': components, 'obstacle_vertices_unchanged': True,
              'clusters': {str(i): m for i, m in clusters.items()},
              'id_mapping': {str(i): r for i, r in mapping.items()}}
    dest.with_suffix('.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k not in ('clusters', 'id_mapping')}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--radius', type=float, default=.05)
    main(p.parse_args())
