import unittest
import lanelet2  # if we fail here, there is something wrong with registration
from lanelet2.core import AttributeMap, getId, BasicPoint2d, Point3d, LineString3d, Lanelet, RegulatoryElement, TrafficLight, LaneletMap, createMapFromLanelets, RightOfWay
from lanelet2.geometry import distance, intersects2d, boundingBox2d, to2D, intersection


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


def getStopLine():
    stopLine = getLineString()
    stopLine.attributes["type"] = "stop_line"
    return stopLine


# need to pass lanelets as arguments, otherwise they are lost, resulting in
# RuntimeError: Nullptr passed to constructor!
def getRightOfWay(row_lanelets, yield_lanelets):
    return RightOfWay(getId(), getAttributes(), row_lanelets, yield_lanelets, getStopLine())


def getLaneletMap():
    lanelet = getLanelet()
    lanelet.addRegulatoryElement(getRegelem())
    return createMapFromLanelets([lanelet])


def getLaneletMapWithRightOfWay():
    yieldLanelet = getLanelet()
    rightOfWayLanelet = getLanelet()
    rightOfWay = getRightOfWay([rightOfWayLanelet], [yieldLanelet])
    rightOfWayLanelet.id = 2000
    rightOfWayLanelet.leftBound.id = 2001
    rightOfWayLanelet.leftBound[0].id = 3000
    rightOfWayLanelet.leftBound[1].id = 3001
    rightOfWayLanelet.rightBound.id = 2002
    rightOfWayLanelet.rightBound[0].id = 3003
    rightOfWayLanelet.rightBound[1].id = 3004
    yieldLanelet.id = 2003
    yieldLanelet.leftBound.id = 2004
    yieldLanelet.leftBound[0].id = 3010
    yieldLanelet.leftBound[1].id = 3011
    yieldLanelet.rightBound.id = 2005
    yieldLanelet.rightBound[0].id = 3013
    yieldLanelet.rightBound[1].id = 3014
    rightOfWay.id = 2006
    rightOfWay.stopLine.id = 2007
    rightOfWay.stopLine[0].id = 3020
    rightOfWay.stopLine[1].id = 3021
    rightOfWayLanelet.addRegulatoryElement(rightOfWay)
    yieldLanelet.addRegulatoryElement(rightOfWay)
    llmap = createMapFromLanelets([rightOfWayLanelet, yieldLanelet])
    llmap.add(rightOfWay)
    return llmap


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


class LaneletReprTestCase(unittest.TestCase):
    def test_lanelet_row_repr(self):
        map = getLaneletMapWithRightOfWay()
        self.assertEqual(len(map.regulatoryElementLayer), 1)
        rightOfWay = map.regulatoryElementLayer[2006]
        rightOfWayLanelet = map.laneletLayer[2000]

        rightOfWayParamsRepr = \
            "RuleParameterMap({" + \
                "'ref_line': [ConstLineString3d(2007, [" + \
                        "ConstPoint3d(3020, 0, 0, 0, AttributeMap({'key': 'value'})), " + \
                        "ConstPoint3d(3021, 0, 0, 0, AttributeMap({'key': 'value'}))" + \
                    "], AttributeMap({'key': 'value', 'type': 'stop_line'}))], " + \
                "'right_of_way': [[id: 2000, left id: 2001, right id: 2002]], " + \
                "'yield': [[id: 2003, left id: 2004, right id: 2005]]" + \
            "})"
        self.assertEqual(repr(rightOfWay.parameters), rightOfWayParamsRepr)
        self.assertEqual(str(rightOfWay.parameters), "{ref_line: 2007 }{right_of_way: 2000 }{yield: 2003 }")

        rightOfWayRepr = "RightOfWay(2006, " + rightOfWayParamsRepr + \
            ", AttributeMap({'key': 'value', 'subtype': 'right_of_way', 'type': 'regulatory_element'}))"
        self.assertEqual(repr(rightOfWay), rightOfWayRepr)
        self.assertEqual(str(rightOfWay), "[id: 2006, parameters: {ref_line: 2007 }{right_of_way: 2000 }{yield: 2003 }]")

        rightOfWayLaneletRepr = \
            "Lanelet(2000, " + \
                "LineString3d(2001, [" + \
                    "Point3d(3000, 0, 0, 0, AttributeMap({'key': 'value'})), " + \
                    "Point3d(3001, 0, 0, 0, AttributeMap({'key': 'value'}))" + \
                "], AttributeMap({'key': 'value'})), " + \
                "LineString3d(2002, [" + \
                    "Point3d(3003, 0, 0, 0, AttributeMap({'key': 'value'})), " + \
                    "Point3d(3004, 0, 0, 0, AttributeMap({'key': 'value'}))" + \
                "], AttributeMap({'key': 'value'})), " + \
                "AttributeMap({'key': 'value'}), " + \
                "[" + rightOfWayRepr + "])"
        self.assertEqual(repr(rightOfWayLanelet), rightOfWayLaneletRepr)
        self.assertEqual(str(rightOfWayLanelet), "[id: 2000, left id: 2001, right id: 2002]")


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

    def test_lanelet_row(self):
        rightOfWayLanelet = getLanelet()
        yieldLanelet = getLanelet()
        rightOfWay = getRightOfWay([rightOfWayLanelet], [yieldLanelet])
        rightOfWayLanelet.addRegulatoryElement(rightOfWay)
        yieldLanelet.addRegulatoryElement(rightOfWay)
        self.assertEqual(len(rightOfWayLanelet.rightOfWay()), 1)
        self.assertEqual(len(rightOfWayLanelet.regulatoryElements), 1)
        self.assertEqual(len(yieldLanelet.rightOfWay()), 1)
        self.assertEqual(len(yieldLanelet.regulatoryElements), 1)
        self.assertEqual(len(rightOfWay.rightOfWayLanelets()), 1)
        self.assertEqual(len(rightOfWay.yieldLanelets()), 1)


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

    def test_intersection_l2l(self):
        point_list = intersection(to2D(getLineString()), to2D(getLineString()))
        self.assertEqual(point_list[0].x, 0.0)
        self.assertEqual(point_list[0].y, 0.0)


if __name__ == '__main__':
    unittest.main()
