"""Independently check the exported binary STL triangles using standard Python."""
import collections
import hashlib
import json
import math
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def cross(a,b):
    return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])


def check(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I',data,80)[0]
    if len(data) != 84+count*50:
        raise ValueError('Invalid STL length: '+path.name)
    edge_counts, directions = collections.Counter(), collections.Counter()
    points, volume, zero_area = set(),0,0
    for index in range(count):
        row = struct.unpack_from('<12fH',data,84+index*50)
        verts = (tuple(row[3:6]),tuple(row[6:9]),tuple(row[9:12]))
        if not all(math.isfinite(value) for vertex in verts for value in vertex):
            raise ValueError('Nonfinite STL coordinates.')
        a,b,c = verts
        normal = cross(tuple(b[i]-a[i] for i in range(3)),tuple(c[i]-a[i] for i in range(3)))
        if sum(v*v for v in normal) <= 1e-20:
            zero_area += 1
        volume += sum(a[i]*cross(b,c)[i] for i in range(3))/6
        points.update(verts)
        for first,last in ((a,b),(b,c),(c,a)):
            key = tuple(sorted((first,last)))
            edge_counts[key] += 1
            directions[key] += 1 if first < last else -1
    bad_edges = sum(v != 2 for v in edge_counts.values())
    inconsistent = sum(v != 0 for v in directions.values())
    if bad_edges or inconsistent or zero_area or volume <= 0:
        raise ValueError('{}: edges {}, orientation {}, zero triangles {}, volume {}'.format(
            path.name,bad_edges,inconsistent,zero_area,volume))
    minimum = [min(p[i] for p in points) for i in range(3)]
    maximum = [max(p[i] for p in points) for i in range(3)]
    return dict(triangles=count,edge_incidence_2=True,consistent_orientation=True,
                positive_volume_mm3=volume,bounds_min_mm=minimum,bounds_max_mm=maximum,
                dimensions_mm=[b-a for a,b in zip(minimum,maximum)],
                sha256=hashlib.sha256(data).hexdigest())


def main():
    result = {p.name:check(p) for p in sorted((ROOT/'stl').glob('*.stl'))}
    expected = json.loads((ROOT/'validation.json').read_text())['printable_meshes']
    if set(result) != {name+'.stl' for name in expected}:
        raise ValueError('Unexpected STL set.')
    for name, values in result.items():
        source = expected[name[:-4]]['volume_mm3']
        if abs(values['positive_volume_mm3']-source) > max(.02,source*.00001):
            raise ValueError('Export volume disagrees with source: '+name)
    (ROOT/'stl_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Verified {} closed, consistently oriented STL exports; source volumes match.'.format(len(result)))


if __name__ == '__main__':
    main()
