#! /usr/bin/env python3 -B

import sys
import json
import numpy as np
from xml.dom import minidom


def parse(infile, outfile):
    doc = minidom.parse(infile) 
    path_strings = [path.getAttribute('d') for path
                    in doc.getElementsByTagName('path')]

    with open(outfile, "wb") as out:
        out.write(len(path_strings).to_bytes(8, "little"))
        for path in path_strings:
            points = path[1:-2].lower().split("l")

            points = [point.split(",") for point in points]
            points = [(float(x), float(y)) for x, y in points]
            points = points[:-1]

            out.write(len(points).to_bytes(8, "little"))
            np_points = np.array(points, 'float32').flatten()
            np_points.tofile(out)

    doc.unlink()

def create_json(infile, outfile):
    doc = minidom.parse(infile) 
    path_strings = [path.getAttribute('d') for path
                    in doc.getElementsByTagName('path')]
    doc.unlink()

    data = {'points_in_screenspace': True, 'points': [], 'polygons' : [[]]}
    for path in path_strings:
        polygon = []

        points = path[1:-2].lower().split("l")
        points = [point.split(",") for point in points]
        points = [[float(x), float(y)] for x, y in points]
        points = points[:-1]

        for point in points:
            polygon.append(len(data['points']))
            data['points'].append(point)
        
        data['polygons'].append(polygon)

    with open(outfile, "w") as out:
        json.dump(data, out)

    print(data)

if __name__ == "__main__":
    infile = sys.argv[1]
    outfile = sys.argv[2]
    create_json(infile, outfile)
