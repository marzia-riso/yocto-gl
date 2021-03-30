#! /usr/bin/env python3 -B

import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from xml.dom import minidom
import xml.etree.ElementTree as ET


def subdivide_bezier(points):
    result = [points[0]]

    def subdivide_polygon(polygon):
        def midpoint(a, b): return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

        Q0 = midpoint(polygon[0], polygon[1])
        Q1 = midpoint(polygon[1], polygon[2])
        Q2 = midpoint(polygon[2], polygon[3])
        R0 = midpoint(Q0, Q1)
        R1 = midpoint(Q1, Q2)
        S = midpoint(R0, R1)
        return [Q0, R0, S, R1, Q2, polygon[3]]

    for i in range(0, len(points)-3, 3):
        print(f'len {len(points)}: {i}, {i+4}')
        polygon = points[i:i+4]
        result += subdivide_polygon(polygon)

    return result


def bezier(points, num_subdivisions):
    result = []
    for _ in range(num_subdivisions):
        result = subdivide_bezier(points)
        points = result[:]
    return result[1:]


def draw_points(points):
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    plt.scatter(x, y)
    plt.show()


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
    root = ET.parse(infile).getroot()
    data = {'screenspace': True, 'polygons': []}

    def parse_path(path_string):
        paths = path_string[1:-1].lower()
        paths = paths.split("zm")
        result = []

        for path in paths:
            final_points = []
            elements = []
            segments = path.split("l")

            for s in segments:
                if "c" in s:
                    points = s.split("c")
                    for p in points:
                        if " " in p:
                            p = p.split(" ")
                            elements.append(p)
                        else:
                            elements.append([p])
                else:
                    elements.append([s])

            for element in elements:
                element = [point.split(",") for point in element]
                element = [(float(x), float(y)) for x, y in element]

                if len(element) == 1:
                    final_points.append(element[0])

                elif len(element) == 3:
                    bezier_points = [final_points[-1]]
                    bezier_points += element

                    assert(len(bezier_points) == 4)
                    bpoints = bezier(bezier_points, 2)

                    final_points += bpoints

                else:
                    print("Boh")

            final_points = final_points[:-1]
            result.append(final_points)

        draw_points(result[0])
        return result

    for element in root:
        if element.tag == "{http://www.w3.org/2000/svg}g":

            transform = element.attrib['transform'][7:-1].split(',')[-2:]
            translation = (float(transform[0]), float(transform[1]))

            for child in element:
                path_points = parse_path(child.attrib['d'])
                for path in path_points:
                    polygon = []
                    for point in path:
                        point = (point[0] + translation[0],
                                 point[1] + translation[1])
                        polygon.append(point)

                    data['polygons'].append(polygon)

        elif element.tag == "{http://www.w3.org/2000/svg}path":
            path_points = parse_path(element.attrib['d'])

            for path in path_points:
                polygon = []
                for point in path:
                    polygon.append(point)
                data['polygons'].append(polygon)

    with open(outfile, "w") as out:
        json.dump(data, out, indent=2)

    # print(data)


if __name__ == "__main__":
    # points = [(0,0), (0,1), (0.5, 0), (1, 0)]
    # p = bezier(points, 8)
    # draw_points(p)

    infile = sys.argv[1]
    outfile = sys.argv[2]
    create_json(infile, outfile)
