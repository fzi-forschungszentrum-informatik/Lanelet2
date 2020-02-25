#!/usr/bin/env python

import lanelet2
import sys
import argparse


def print_layer(layer, layerName):
    print("IDs in " + layerName)
    print(sorted([elem.id for elem in layer]))


parser = argparse.ArgumentParser()
parser.add_argument("filename", help="Path to the input osm file")
parser.add_argument("--has-id", type=int, help="Check if the ID is present in the map")
args = parser.parse_args()

proj = lanelet2.projection.MercatorProjector(lanelet2.io.Origin(49, 8))
map = lanelet2.io.load(args.filename, proj)

layers = {"Points": map.pointLayer, "Line Strings": map.lineStringLayer, "Polygons": map.polygonLayer,
          "Lanelets": map.laneletLayer, "Areas": map.areaLayer, "Regulatory Elements": map.regulatoryElementLayer}

for layer_name, layer in layers.iteritems():
    if not args.has_id:
        print_layer(layer, layer_name)
    else:
        for elem in layer:
            if elem.id == args.has_id:
                print("Found ID " + str(elem.id) + " in layer " + layer_name)
                sys.exit(0)
if args.has_id:
    print("ID " + args.has_id + " not in map")
