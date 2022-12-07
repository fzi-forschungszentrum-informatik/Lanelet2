#!/usr/bin/env python
import os
import tempfile

import lanelet2
from lanelet2.core import (AllWayStop, AttributeMap, BasicPoint2d,
                           BoundingBox2d, Lanelet, LaneletMap,
                           LaneletWithStopLine, LineString3d, Point2d, Point3d,
                           RightOfWay, TrafficLight, getId)
from lanelet2.projection import (UtmProjector, MercatorProjector,
                                 LocalCartesianProjector, GeocentricProjector)


example_file = os.path.join(os.path.dirname(os.path.abspath(
    __file__)), "../../lanelet2_maps/res/mapping_example.osm")
if not os.path.exists(example_file):
    # location after installing
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "../../share/lanelet2_maps/res/mapping_example.osm")


def tutorial():
    # We do our best to keep the python interface in sync with the c++ implementation. As a rule of thumb: Everything
    # you can do in c++ works pretty similar in python, we just "pythonized" some things. Therefore this tutorial only
    # shows you just the most important things and the things that are different from C++. For the rest have a look
    # at the c++ tutorial.
    part1primitives()
    part2regulatory_elements()
    part3lanelet_map()
    part4reading_and_writing()
    part5traffic_rules()
    part6routing()


def part1primitives():
    # Primitives work very similar to c++, except that the data can be accessed as properties instead of functions
    p = Point3d(getId(), 0, 0, 0)
    assert p.x == 0
    p.id = getId()
    p.attributes["key"] = "value"
    assert "key" in p.attributes
    assert p.attributes["key"] == "value"

    # the 2d/3d mechanics work too
    p2d = lanelet2.geometry.to2D(p)

    # all (common) geometry calculations are available as well:
    p2 = Point3d(getId(), 1, 0, 0)
    assert lanelet2.geometry.distance(p, p2) == 1
    assert lanelet2.geometry.distance(p2d, Point2d(getId(), 1, 0, 1)) == 1

    # linestrings work conceptually similar to a list (but they only accept points, of course)
    ls = LineString3d(getId(), [p, p2])
    assert ls[0] == p
    assert ls[-1] == p2
    assert p in ls
    for pt in ls:
        assert pt.y == 0

    ls_inv = ls.invert()
    assert ls_inv[0] == p2
    ls.append(Point3d(getId(), 2, 0, 0))
    del ls[2]


def part2regulatory_elements():
    # regulatory elements profit from pythons type system

    # TrafficLight
    lanelet = get_a_lanelet()
    light = get_linestring_at_y(3)
    traffic_light_regelem = TrafficLight(getId(), AttributeMap(), [light])
    lanelet.addRegulatoryElement(traffic_light_regelem)
    assert traffic_light_regelem in lanelet.regulatoryElements
    lights = [regelem for regelem in lanelet.regulatoryElements if isinstance(
        regelem, TrafficLight)]
    assert traffic_light_regelem in lights
    assert light in lights[0].trafficLights

    # RightOfWay
    stop_linestring = get_linestring_at_y(0)
    right_of_way_lanelets = [get_a_lanelet(), get_a_lanelet(1)]
    yielding_lanelets = [get_a_lanelet(2)]
    right_of_way_regelem = RightOfWay(getId(),
                                      AttributeMap(),
                                      right_of_way_lanelets,
                                      yielding_lanelets,
                                      stop_linestring)
    map = LaneletMap()
    map.add(yielding_lanelets[0])
    map.add(right_of_way_lanelets[0])
    map.add(right_of_way_lanelets[1])
    # must add to the map explicitly
    map.add(right_of_way_regelem)
    assert right_of_way_regelem in map.regulatoryElementLayer
    rightOfWays = [regelem for regelem in map.regulatoryElementLayer
                   if isinstance(regelem, RightOfWay)]
    assert right_of_way_regelem in rightOfWays
    # must have the circular reference from the yielding lanelet
    # otherwise, the last assertion will fail
    # this should have been automatically inferred by the regulatoryElements
    # getter function
    yielding_lanelets[0].addRegulatoryElement(right_of_way_regelem)
    # This regulatory element should affect the yielding lanelet
    assert right_of_way_regelem in yielding_lanelets[0].regulatoryElements

    # AllWayStop
    lanelets_with_stop_lines = [
        LaneletWithStopLine(get_a_lanelet(), get_linestring_at_y(0)),
        LaneletWithStopLine(get_a_lanelet(1), get_linestring_at_y(1)),
        LaneletWithStopLine(get_a_lanelet(2), get_linestring_at_y(2)),
        LaneletWithStopLine(get_a_lanelet(3), get_linestring_at_y(3))
    ]
    map = LaneletMap()
    # add the lanelets to the map, access stop line
    for lanelet_with_stop_line in lanelets_with_stop_lines:
        map.add(lanelet_with_stop_line.lanelet)
        lanelet_with_stop_line.stopLine
    all_way_stop_regelem = AllWayStop(getId(),
                                      AttributeMap(),
                                      lanelets_with_stop_lines)
    # must add to the map explicitly
    map.add(all_way_stop_regelem)
    assert all_way_stop_regelem in map.regulatoryElementLayer
    allWayStops = [regelem for regelem in map.regulatoryElementLayer
                   if isinstance(regelem, AllWayStop)]
    assert all_way_stop_regelem in allWayStops
    # must have the circular reference from each yielding lanelet
    # otherwise, the last assertion will fail
    # this should have been automatically inferred by the regulatoryElements
    # getter function
    for lanelet_with_stop_line in lanelets_with_stop_lines:
        lanelet_with_stop_line.lanelet.addRegulatoryElement(
            all_way_stop_regelem)
    # This regulatory element should affect each yielding lanelet
    for lanelet_with_stop_line in lanelets_with_stop_lines:
        assert all_way_stop_regelem in lanelet_with_stop_line.lanelet.regulatoryElements


def part3lanelet_map():
    # lanelets map work just as you would expect:
    map = LaneletMap()
    lanelet = get_a_lanelet()
    map.add(lanelet)
    assert lanelet in map.laneletLayer
    assert map.pointLayer
    assert not map.areaLayer
    assert len(map.pointLayer.nearest(BasicPoint2d(0, 0), 1)) == 1
    searchBox = BoundingBox2d(BasicPoint2d(0, 0), BasicPoint2d(2, 2))
    assert len(map.pointLayer.search(searchBox)) > 1

    # you can also create a map from a list of primitives (replace Lanelets by the other types)
    mapBulk = lanelet2.core.createMapFromLanelets([get_a_lanelet()])
    assert len(mapBulk.laneletLayer) == 1


def part4reading_and_writing():
    # there are two ways of loading/writing a lanelet map: a robust one and an normal one. The robust one returns found
    # issues as extra return parameter
    map = LaneletMap()
    lanelet = get_a_lanelet()
    map.add(lanelet)
    path = os.path.join(tempfile.mkdtemp(), 'mapfile.osm')
    # Select a suitable projector depending on the data source
    ## UtmProjector: (0,0,0) is at the provided lat/lon on the WGS84 ellipsoid
    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
    ## MarcatorProjector: (0,0,0) is at the provided lat/lon on the mercator cylinder
    projector = MercatorProjector(lanelet2.io.Origin(49, 8.4))
    ## LocalCartesianProjector: (0,0,0) is at the provided origin (including elevation)
    projector = LocalCartesianProjector(lanelet2.io.Origin(49, 8.4, 123))

    # Writing the map to a file
    ## 1. Write with the given projector and use default parameters
    lanelet2.io.write(path, map, projector)

    ## 2. Write and get the possible errors
    write_errors = lanelet2.io.writeRobust(path, map, projector)
    assert not write_errors

    ## 3. Write using the default spherical mercator projector at the giver origin
    ## This was the default projection in Lanelet1
    lanelet2.io.write(path, map, lanelet2.io.Origin(49, 8.4))

    ## 4. Write using the given projector and override the default values of the optional parameters for JOSM
    params = {
               "josm_upload": "true",          # value for the attribute "upload", default is "false"
               "josm_format_elevation": "true"  # whether to limit up to 2 decimals, default is the same as for lat/lon
             };
    lanelet2.io.write(path, map, projector, params)

    # Loading the map from a file
    loadedMap, load_errors = lanelet2.io.loadRobust(path, projector)
    assert not load_errors
    assert loadedMap.laneletLayer.exists(lanelet.id)


def part5traffic_rules():
    # this is just as you would expect
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    lanelet = get_a_lanelet()
    lanelet.attributes["vehicle"] = "yes"
    assert traffic_rules.canPass(lanelet)
    assert traffic_rules.speedLimit(lanelet).speedLimit > 1


def part6routing():
    # and this as well
    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
    map = lanelet2.io.load(example_file, projector)
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(map, traffic_rules)
    lanelet = map.laneletLayer[4984315]
    toLanelet = map.laneletLayer[2925017]
    assert graph.following(lanelet)
    assert len(graph.reachableSet(lanelet, 100, 0)) > 10
    assert len(graph.possiblePaths(lanelet, 100, 0, False)) == 1

    # here we query a route through the lanelets and get all the vehicle lanelets that conflict with the shortest path
    # in that route
    route = graph.getRoute(lanelet, toLanelet)
    path = route.shortestPath()
    confLlts = [llt for llt in route.allConflictingInMap() if llt not in path]
    assert len(confLlts) > 0

    # for more complex queries, you can use the forEachSuccessor function and pass it a function object
    assert hasPathFromTo(graph, lanelet, toLanelet)


def hasPathFromTo(graph, start, target):
    class TargetFound(BaseException):
        pass

    def raiseIfDestination(visitInformation):
        # this function is called for every successor of lanelet with a LaneletVisitInformation object.
        # if the function returns true, the search continues with the successors of this lanelet.
        # Otherwise, the followers will not be visited through this lanelet, but could still be visited through
        # other lanelets.
        if visitInformation.lanelet == target:
            raise TargetFound()
        else:
            return True
    try:
        graph.forEachSuccessor(start, raiseIfDestination)
        return False
    except TargetFound:
        return True


def get_linestring_at_x(x):
    return LineString3d(getId(), [Point3d(getId(), x, i, 0) for i in range(0, 3)])


def get_linestring_at_y(y):
    return LineString3d(getId(), [Point3d(getId(), i, y, 0) for i in range(0, 3)])


def get_a_lanelet(index=0):
    return Lanelet(getId(),
                   get_linestring_at_y(2+index),
                   get_linestring_at_y(0+index))


if __name__ == '__main__':
    tutorial()
