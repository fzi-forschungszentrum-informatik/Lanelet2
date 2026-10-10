import unittest
from lanelet2.core import AttributeMap, Point3d, LineString3d, Lanelet, createMapFromLanelets
from lanelet2.traffic_rules import create as createTrafficRules, Locations, Participants
from lanelet2.routing import RoutingGraph


def makeSkipOverMap():
    # Driving +x, y increases to the left:
    #   v2 | bike.leftBound | bike | bike.rightBound | v1
    p1, p2 = Point3d(1, 0, 0, 0), Point3d(2, 10, 0, 0)
    p3, p4 = Point3d(3, 0, 2, 0), Point3d(4, 10, 2, 0)
    p5, p6 = Point3d(5, 0, 4, 0), Point3d(6, 10, 4, 0)
    p7, p8 = Point3d(7, 0, 6, 0), Point3d(8, 10, 6, 0)
    v1Right = LineString3d(10, [p1, p2])
    sharedV1Bike = LineString3d(11, [p3, p4], AttributeMap({"type": "bike_marking", "subtype": "dashed"}))
    sharedBikeV2 = LineString3d(12, [p5, p6], AttributeMap({"type": "bike_marking", "subtype": "dashed"}))
    v2Left = LineString3d(13, [p7, p8])
    road = AttributeMap({"subtype": "road"})
    bikeAttr = AttributeMap({"subtype": "bicycle_lane"})
    v1 = Lanelet(21, sharedV1Bike, v1Right, road)
    bike = Lanelet(22, sharedBikeV2, sharedV1Bike, bikeAttr)
    v2 = Lanelet(23, v2Left, sharedBikeV2, road)
    return v1, bike, v2, createMapFromLanelets([v1, bike, v2])


class RoutingGraphConfigurationTestCase(unittest.TestCase):
    def setUp(self):
        self.v1, self.bike, self.v2, self.map = makeSkipOverMap()
        self.rules = createTrafficRules(Locations.Germany, Participants.Vehicle)

    def test_bicycle_lane_skip_over_disabled_by_default(self):
        graph = RoutingGraph(self.map, self.rules)
        self.assertIsNone(graph.left(self.v1))
        self.assertIsNone(graph.right(self.v2))

    def test_bicycle_lane_skip_over_explicitly_disabled(self):
        graph = RoutingGraph(self.map, self.rules,
                             configuration={"allow_lane_change_across_bicycle_lane": "false"})
        self.assertIsNone(graph.left(self.v1))

    def test_bicycle_lane_skip_over_enabled_via_configuration(self):
        graph = RoutingGraph(self.map, self.rules,
                             configuration={"allow_lane_change_across_bicycle_lane": "true"})
        self.assertEqual(graph.left(self.v1), self.v2)
        self.assertEqual(graph.right(self.v2), self.v1)


if __name__ == "__main__":
    unittest.main()
