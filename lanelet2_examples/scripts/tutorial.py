#!/usr/bin/env python
import lanelet2
import tempfile
import os
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
from lanelet2.projection import UtmProjector

example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../lanelet2_maps/res/mapping_example.osm")


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
    lanelet = get_a_lanelet()
    light = get_linestring_at_y(3)
    regelem = TrafficLight(getId(), AttributeMap(), [light])
    lanelet.addRegulatoryElement(regelem)
    assert regelem in lanelet.regulatoryElements
    lights = [regelem for regelem in lanelet.regulatoryElements if isinstance(regelem, TrafficLight)]
    assert regelem in lights
    assert light in lights[0].trafficLights


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
    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
    lanelet2.io.write(path, map, projector)
    mapLoad, errors = lanelet2.io.loadRobust(path, projector)
    assert not errors
    assert mapLoad.laneletLayer.exists(lanelet.id)


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


def get_a_lanelet():
    return Lanelet(getId(), get_linestring_at_y(2), get_linestring_at_y(0))


if __name__ == '__main__':
    tutorial()
