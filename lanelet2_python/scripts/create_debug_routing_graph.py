#!/usr/bin/env python

import lanelet2
import sys
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="Path to the input osm file")
    parser.add_argument("output", help="Path to resulting debug routing graph")
    parser.add_argument("--participant", help="traffic participant type (one of vehicle, bicycle, pedestrian, train", type=str, required=True, default="vehicle")
    args = parser.parse_args()

    rules_map = {"vehicle" : lanelet2.traffic_rules.Participants.Vehicle,
                 "bicycle" : lanelet2.traffic_rules.Participants.Bicycle,
                 "pedestrian" : lanelet2.traffic_rules.Participants.Pedestrian,
                 "train" : lanelet2.traffic_rules.Participants.Train}
    proj = lanelet2.projection.UtmProjector(lanelet2.io.Origin(49, 8))
    laneletmap = lanelet2.io.load(args.filename, proj)

    routing_cost = lanelet2.routing.RoutingCostDistance(0.)  # zero cost for lane changes
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  rules_map[args.participant])
    graph = lanelet2.routing.RoutingGraph(laneletmap, traffic_rules, [routing_cost])
    debug_map = graph.getDebugLaneletMap()

    lanelet2.io.write(args.output, debug_map, proj)
