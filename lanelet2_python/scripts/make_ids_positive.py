#!/usr/bin/env python

import lanelet2
import sys
import argparse


def make_positive(layer):
    for elem in layer:
        if elem.id < 0:
            elem.id = layer.uniqueId()


parser = argparse.ArgumentParser()
parser.add_argument("filename", help="Path to the input osm file")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("output", help="Path to results", nargs='?')
group.add_argument("-i", "--inplace", action="store_true", help="Overwrite input file")
args = parser.parse_args()

if args.inplace:
    args.output = args.filename

proj = lanelet2.projection.MercatorProjector(lanelet2.io.Origin(49, 8))
map = lanelet2.io.load(args.filename, proj)


make_positive(map.pointLayer)
make_positive(map.lineStringLayer)
make_positive(map.polygonLayer)
make_positive(map.laneletLayer)
make_positive(map.areaLayer)
make_positive(map.regulatoryElementLayer)

lanelet2.io.write(args.output, map, proj)
