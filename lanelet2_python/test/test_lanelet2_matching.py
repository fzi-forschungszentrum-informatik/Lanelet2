import unittest
import lanelet2  # if we fail here, there is something wrong with lanelet2 registration
from lanelet2.core import AttributeMap, getId, BasicPoint2d, Point3d, LineString3d, Lanelet, RegulatoryElement, TrafficLight, LaneletMap, createMapFromLanelets, ConstLanelet, ConstLineString3d
from lanelet2.geometry import distance, intersects2d, boundingBox2d, to2D, equals
from lanelet2.matching import Pose2d, getDeterministicMatches, getProbabilisticMatches, Object2d, ObjectWithCovariance2d, PositionCovariance2d, removeNonRuleCompliantMatches
from lanelet2.matching import ConstLaneletMatch, ConstLaneletMatchProbabilistic


def get_sample_lanelet_map():
    mymap = LaneletMap()
    x_left = 2
    x_right = 0
    ls_left = LineString3d(getId(), [Point3d(getId(), x_left, i, 0) for i in range(0, 3)])
    ls_right = LineString3d(getId(), [Point3d(getId(), x_right, i, 0) for i in range(0, 3)])
    llet = Lanelet(getId(), ls_left, ls_right)
    mymap.add(llet)
    return mymap


class ObjectsApiTestCase(unittest.TestCase):
    def test_default_init(self):
        obj_2d = Object2d()
        self.assertTrue(isinstance(obj_2d, Object2d))
        self.assertEqual(len(obj_2d.absoluteHull), 0)

        obj_with_cov_2d = ObjectWithCovariance2d()
        self.assertTrue(isinstance(obj_with_cov_2d, ObjectWithCovariance2d))
        self.assertEqual(len(obj_with_cov_2d.absoluteHull), 0)
        self.assertEqual(obj_with_cov_2d.vonMisesKappa, 0)

        pos_cov_2d = PositionCovariance2d()
        self.assertTrue(isinstance(pos_cov_2d, PositionCovariance2d))

        pose_2d = Pose2d()
        self.assertTrue(isinstance(pose_2d, Pose2d))

    def test_custom_init(self):
        obj_2d = Object2d(1, Pose2d(), [])
        self.assertTrue(isinstance(obj_2d, Object2d))
        self.assertEqual(len(obj_2d.absoluteHull), 0)

        obj_with_cov_2d = ObjectWithCovariance2d(1, Pose2d(), [], PositionCovariance2d(), 2)
        self.assertTrue(isinstance(obj_with_cov_2d, ObjectWithCovariance2d))
        self.assertEqual(len(obj_with_cov_2d.absoluteHull), 0)
        self.assertEqual(obj_with_cov_2d.vonMisesKappa, 2)

        pos_cov_2d = PositionCovariance2d(1., 2., 3.)
        self.assertTrue(isinstance(pos_cov_2d, PositionCovariance2d))
        self.assertEqual("1 3\n3 2", str(pos_cov_2d))

        pose_2d = Pose2d(1., 2., 3.)
        self.assertTrue(isinstance(pose_2d, Pose2d))
        self.assertEqual("x: 1\ny:  2\nphi: 3", str(pose_2d))


class MatchingApiTestCase(unittest.TestCase):
    def test_matching(self):
        mymap = get_sample_lanelet_map()

        obj = Object2d(1, Pose2d(1, 1, 0), [])
        obj_with_cov = ObjectWithCovariance2d(1, Pose2d(1, 1, 0), [], PositionCovariance2d(1., 1., 0.), 2)

        obj_matches = getDeterministicMatches(mymap, obj, 1.)
        self.assertEqual(len(obj_matches), 2)  # lanelet in both directions
        self.assertTrue(isinstance(obj_matches[0], ConstLaneletMatch))

        obj_with_cov_matches = getProbabilisticMatches(mymap, obj_with_cov, 1.)
        self.assertEqual(len(obj_with_cov_matches), 2)  # lanelet in both directions
        self.assertTrue(isinstance(obj_with_cov_matches[0], ConstLaneletMatchProbabilistic))

        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)

        obj_matches_rule_compliant = removeNonRuleCompliantMatches(obj_matches, traffic_rules)
        self.assertEqual(len(obj_matches_rule_compliant), 1)  # lanelet only in one direction
        self.assertTrue(isinstance(obj_matches_rule_compliant[0], ConstLaneletMatch))

        obj_with_cov_matches_rule_compliant = removeNonRuleCompliantMatches(obj_with_cov_matches, traffic_rules)
        self.assertEqual(len(obj_with_cov_matches_rule_compliant), 1)  # lanelet only in one direction
        self.assertTrue(isinstance(obj_with_cov_matches_rule_compliant[0], ConstLaneletMatchProbabilistic))
        self.assertTrue(isinstance(obj_with_cov_matches_rule_compliant[0].lanelet, ConstLanelet))
        self.assertEqual(obj_with_cov_matches_rule_compliant[0].distance, 0)
        self.assertTrue(isinstance(obj_with_cov_matches_rule_compliant[0].mahalanobisDistSq, float))

        # empty matches list
        empty_matches_rule_compliant = removeNonRuleCompliantMatches(list(), traffic_rules)
        self.assertEqual(len(empty_matches_rule_compliant), 0)

        # list of neither matches nor probabilistic matches
        self.assertRaises(RuntimeError, removeNonRuleCompliantMatches, [obj], traffic_rules)


if __name__ == '__main__':
    unittest.main()
