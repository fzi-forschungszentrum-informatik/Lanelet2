#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/GermanTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <utility>

#include "lanelet2_ml_converter/Forward.h"
#include "lanelet2_ml_converter/Utils.h"

/// The coordinates and relations for this test can be found in "LaneletTestMap.xml" which can be viewed in
/// https://www.draw.io
namespace lanelet {
namespace ml_converter {
namespace tests {

class MapTestData {
 public:
  MapTestData() {
    initPoints();
    initLineStrings();
    initLanelets();
    laneletMap = std::make_shared<LaneletMap>(lanelets, areas, std::unordered_map<Id, RegulatoryElementPtr>(),
                                              std::unordered_map<Id, Polygon3d>(), lines, points);
  }

  void addPoint(double x, double y, double z) {
    points.insert(std::pair<Id, Point3d>(pointId, Point3d(pointId, x, y, z)));
    pointId++;
  }

  void addLine(const Points3d& points) {
    lines.insert(std::pair<Id, LineString3d>(lineId, LineString3d(lineId, points)));
    lineId++;
  }

  void addLaneletVehicle(const LineString3d& left, const LineString3d& right) {
    Lanelet ll{laneletId, left, right};
    ll.setAttribute(AttributeName::Subtype, AttributeValueString::Road);
    lanelets.insert(std::make_pair(laneletId, ll));
    laneletId++;
  }

  Id pointId{0};
  Id lineId{1000};
  Id laneletId{2000};
  std::unordered_map<Id, Lanelet> lanelets;
  std::unordered_map<Id, Point3d> points;
  std::unordered_map<Id, LineString3d> lines;
  std::unordered_map<Id, Area> areas;
  LaneletMapPtr laneletMap;

 private:
  void initPoints() {
    points.clear();
    addPoint(0.0, 0.0, 1.0);    // p0
    addPoint(2.0, 0.0, 1.0);    // p1
    addPoint(4.0, 0.0, 1.0);    // p2
    addPoint(6.0, 0.0, 1.0);    // p3
    addPoint(8.0, 0.0, 1.0);    // p4
    addPoint(10.0, 0.0, 1.0);   // p5
    addPoint(12.0, 0.0, 1.0);   // p6
    addPoint(14.0, 0.0, 1.0);   // p7
    addPoint(6.0, -1.0, 1.0);   // p8
    addPoint(8.0, -1.0, 1.0);   // p9
    addPoint(0.0, -2.0, 1.0);   // p10
    addPoint(2.0, -2.0, 1.0);   // p11
    addPoint(4.0, -2.0, 1.0);   // p12
    addPoint(6.0, -2.0, 1.0);   // p13
    addPoint(8.0, -2.0, 1.0);   // p14
    addPoint(10.0, -2.0, 1.0);  // p15
    addPoint(12.0, -2.0, 1.0);  // p16
    addPoint(14.0, -2.0, 1.0);  // p17
    addPoint(6.0, -3.0, 1.0);   // p18
    addPoint(8.0, -3.0, 1.0);   // p19
    addPoint(0.0, -4.0, 1.0);   // p20
    addPoint(2.0, -4.0, 1.0);
    addPoint(4.0, -4.0, 1.0);
    addPoint(6.0, -4.0, 1.0);
    addPoint(8.0, -4.0, 1.0);
    addPoint(10.0, -4.0, 1.0);
    addPoint(12.0, -4.0, 1.0);
    addPoint(14.0, -4.0, 1.0);  // p27
    addPoint(5.0, -5.0, 1.0);
    addPoint(7.0, -5.0, 1.0);
    addPoint(9.0, -5.0, 1.0);  // p30
    addPoint(5.0, -7.0, 1.0);
    addPoint(7.0, -7.0, 1.0);
    addPoint(9.0, -7.0, 1.0);  // p33
    addPoint(5.0, -9.0, 1.0);
    addPoint(7.0, -9.0, 1.0);
    addPoint(9.0, -9.0, 1.0);  // p36
    addPoint(5.0, -11.0, 1.0);
    addPoint(7.0, -11.0, 1.0);
    addPoint(9.0, -11.0, 1.0);  // p39
  }

  void initLineStrings() {
    lines.clear();
    addLine(Points3d{points.at(5), points.at(6), points.at(7)});  // l1000
    lines.at(1000).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(2), points.at(3), points.at(4), points.at(5)});  // l1001
    lines.at(1001).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(0), points.at(1), points.at(2)});  // l1002
    lines.at(1002).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(15), points.at(16), points.at(17)});  // l1003
    lines.at(1003).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1003).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(12), points.at(13), points.at(14), points.at(15)});  // l1004
    lines.at(1004).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1004).setAttribute(AttributeName::Subtype, AttributeValueString::Solid);
    addLine(Points3d{points.at(10), points.at(11), points.at(12)});  // l1005
    lines.at(1005).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1005).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);

    addLine(Points3d{points.at(20), points.at(21), points.at(22)});  // l1006
    lines.at(1006).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(22), points.at(23), points.at(24), points.at(25)});  // l1007
    lines.at(1007).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1007).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(25), points.at(26), points.at(27)});  // l1008
    lines.at(1008).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(30), points.at(25)});  // l1009
    lines.at(1009).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(22), points.at(28)});  // l1010
    lines.at(1010).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(28), points.at(31)});  // l1011
    lines.at(1011).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(29), points.at(32)});  // l1012
    lines.at(1012).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1012).setAttribute(AttributeName::Subtype, AttributeValueString::Solid);
    addLine(Points3d{points.at(33), points.at(30)});  // l1013
    lines.at(1013).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(31), points.at(34), points.at(37)});  // l1014
    lines.at(1014).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(32), points.at(35), points.at(38)});  // l1015
    lines.at(1015).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1015).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(39), points.at(36), points.at(33)});  // l1016
    lines.at(1016).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(12), points.at(18), points.at(29)});  // l1017
    lines.at(1017).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(29), points.at(19), points.at(15)});  // l1018
    lines.at(1018).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
  }
  void initLanelets() {
    lanelets.clear();
    addLaneletVehicle(lines.at(1002), lines.at(1005));           // ll2000
    addLaneletVehicle(lines.at(1001), lines.at(1004));           // ll2001
    addLaneletVehicle(lines.at(1000), lines.at(1003));           // ll2002
    addLaneletVehicle(lines.at(1005), lines.at(1006));           // ll2003
    addLaneletVehicle(lines.at(1004), lines.at(1007));           // ll2004
    addLaneletVehicle(lines.at(1003), lines.at(1008));           // ll2005
    addLaneletVehicle(lines.at(1012), lines.at(1011));           // ll2006
    addLaneletVehicle(lines.at(1012).invert(), lines.at(1013));  // ll2007
    addLaneletVehicle(lines.at(1015), lines.at(1014));           // ll2008
    addLaneletVehicle(lines.at(1015).invert(), lines.at(1016));  // ll2009
    addLaneletVehicle(lines.at(1017), lines.at(1010));           // ll2010
    addLaneletVehicle(lines.at(1018), lines.at(1009));           // ll2011
  }
};

namespace {                   // NOLINT
static MapTestData testData;  // NOLINT
}  // namespace

class MLConverterTest : public ::testing::Test {
 public:
  const std::unordered_map<Id, Lanelet>& lanelets{testData.lanelets};
  const std::unordered_map<Id, Point3d>& points{testData.points};
  const std::unordered_map<Id, LineString3d>& lines{testData.lines};
  const LaneletMapConstPtr laneletMap{testData.laneletMap};
  const BasicPoint3d centerBbox{5, 5, 0};
  const double extentLongitudinalBbox{15};
  const double extentLateralBbox{10};
  const double yawBbox{M_PI / 2.0};
  OrientedRect bbox;
  MLConverterTest() : bbox{getRotatedRect(centerBbox, extentLongitudinalBbox, extentLateralBbox, yawBbox, true)} {}
};

}  // namespace tests
}  // namespace ml_converter
}  // namespace lanelet
