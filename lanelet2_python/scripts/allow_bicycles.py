#!/usr/bin/env python

import lanelet2
import sys
import argparse


def allow_bicycles(layer):
    for elem in layer:
        if "vehicle" in elem.attributes and elem.attributes["vehicle"] == "yes":
            if "subtype" in elem.attributes and elem.attributes["subtype"] == "highway":
                return
            else:
                elem.attributes["bicycle"] = "yes"


def bike_to_bicycle(layer):
    for elem in layer:
        if "bike" in elem.attributes:
            elem.attributes["bicycle"] = elem.attributes["bike"]
            del elem.attributes["bike"]


parser = argparse.ArgumentParser()
parser.add_argument("filename", help="Path to the input osm file")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("output", help="Path to results", nargs='?')
group.add_argument("-i", "--inplace", action="store_true", help="Overwrite input file")
args = parser.parse_args()

if args.inplace:
    args.output = args.filename

map = lanelet2.io.load(args.filename)

allow_bicycles(map.laneletLayer)
bike_to_bicycle(map.laneletLayer)

lanelet2.io.write(args.output, map)
