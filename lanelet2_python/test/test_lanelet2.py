import unittest
import lanelet2  # if we fail here, there is something wrong with registration
from lanelet2.core import AttributeMap, getId, BasicPoint2d, Point3d, LineString3d, Lanelet, RegulatoryElement, TrafficLight, LaneletMap, createMapFromLanelets
from lanelet2.geometry import distance, intersects2d, boundingBox2d, to2D


def getAttributes():
    return AttributeMap({"key": "value"})


def getPoint():
    return Point3d(getId(), 0, 0, 0, getAttributes())


def getLineString():
    return LineString3d(getId(), [getPoint(), getPoint()], getAttributes())


def getLanelet():
    return Lanelet(getId(), getLineString(), getLineString(), getAttributes())


def getRegelem():
    return TrafficLight(getId(), AttributeMap(), [getLineString()], getLineString())


def getLaneletMap():
    lanelet = getLanelet()
    lanelet.addRegulatoryElement(getRegelem())
    return createMapFromLanelets([lanelet])


def checkPrimitiveId(testClass, primitive):
    primitive.id = 30
    testClass.assertEqual(primitive.id, 30)


def checkPrimitiveAttributes(testClass, primitive):
    lenBefore = len(primitive.attributes)
    primitive.attributes["newkey"] = "newvalue"
    testClass.assertEqual(lenBefore + 1, len(primitive.attributes))
    testClass.assertTrue("newkey" in primitive.attributes)
    testClass.assertEqual(primitive.attributes["newkey"], "newvalue")
    del primitive.attributes["newkey"]
    testClass.assertFalse("newkey" in primitive.attributes)


class LaneletApiTestCase(unittest.TestCase):
    def test_lanelet_id(self):
        checkPrimitiveId(self, getLanelet())

    def test_lanelet_attributes(self):
        checkPrimitiveAttributes(self, getLanelet())

    def test_lanelet_modification(self):
        lanelet = getLanelet()
        bound = getLineString()
        lanelet.leftBound = bound
        self.assertEqual(bound, lanelet.leftBound)

    def test_lanelet_regelem(self):
        regelem = getRegelem()
        llt = getLanelet()
        llt.addRegulatoryElement(regelem)
        self.assertEqual(len(llt.trafficLights()), 1)
        self.assertEqual(len(llt.trafficSigns()), 0)
        self.assertEqual(len(llt.regulatoryElements), 1)


class LaneletMapApiTestCase(unittest.TestCase):
    def test_lanelet_map_basic(self):
        map = getLaneletMap()
        self.assertEqual(len(map.laneletLayer), 1)
        self.assertEqual(len(map.regulatoryElementLayer), 1)

    def test_lanelet_map_search(self):
        map = getLaneletMap()
        nearest = map.laneletLayer.nearest(BasicPoint2d(0, 0), 1)
        self.assertEqual(len(nearest), 1)
        self.assertTrue(map.laneletLayer.exists(nearest[0].id))


class GeometryApiTestCase(unittest.TestCase):
    def test_distance_p2p(self):
        self.assertEqual(distance(getPoint(), getPoint()), 0)

    def test_distance_basic_p2p(self):
        self.assertEqual(distance(getPoint().basicPoint(), getPoint()), 0)

    def test_distance_l2l(self):
        self.assertEqual(distance(getLineString(), getLineString()), 0)

    def test_distance_llt2llt(self):
        self.assertEqual(distance(getLanelet(), getLanelet()), 0)

    def test_intersects_l2l(self):
        self.assertTrue(intersects2d(to2D(getLineString()), to2D(getLineString())))

    def test_bounding_box_line(self):
        bbox = boundingBox2d(to2D(getLineString()))
        self.assertEqual(bbox.min.x, 0)


if __name__ == '__main__':
    unittest.main()
