#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/GermanTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <utility>

#include "lanelet2_routing/Forward.h"
#include "lanelet2_routing/RoutingGraph.h"

/// The coordinates and relations for this test can be found in "LaneletTestMap.xml" which can be viewed in
/// https://www.draw.io
namespace lanelet {
namespace routing {
namespace tests {

inline RoutingGraphPtr setUpGermanVehicleGraph(LaneletMap& map, double laneChangeCost = 2.,
                                               double participantHeight = 2., double minLaneChangeLength = 0.) {
  traffic_rules::TrafficRulesPtr trafficRules{traffic_rules::TrafficRulesFactory::create(
      Locations::Germany, Participants::Vehicle, traffic_rules::TrafficRules::Configuration())};
  RoutingCostPtrs costPtrs{std::make_shared<RoutingCostDistance>(laneChangeCost, minLaneChangeLength),
                           std::make_shared<RoutingCostTravelTime>(laneChangeCost)};
  RoutingGraph::Configuration configuration;
  configuration.insert(std::make_pair(RoutingGraph::ParticipantHeight, participantHeight));
  return RoutingGraph::build(map, *trafficRules, costPtrs, configuration);
}

inline RoutingGraphPtr setUpGermanPedestrianGraph(LaneletMap& map, double laneChangeCost = 2.) {
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian)};
  RoutingCostPtrs costPtrs{std::make_shared<RoutingCostDistance>(laneChangeCost)};
  return RoutingGraph::build(map, *trafficRules, costPtrs);
}

inline RoutingGraphPtr setUpGermanBicycleGraph(LaneletMap& map, double laneChangeCost = 2.) {
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle)};
  RoutingCostPtrs costPtrs{std::make_shared<RoutingCostDistance>(laneChangeCost)};
  return RoutingGraph::build(map, *trafficRules, costPtrs);
}

class RoutingGraphTestData {
 public:
  RoutingGraphTestData() {
    initPoints();
    initLineStrings();
    initLanelets();
    initAreas();
    laneletMap = std::make_shared<LaneletMap>(lanelets, areas, std::unordered_map<Id, RegulatoryElementPtr>(),
                                              std::unordered_map<Id, Polygon3d>(), lines, points);
    vehicleGraph = setUpGermanVehicleGraph(*laneletMap, laneChangeCost);
    pedestrianGraph = setUpGermanPedestrianGraph(*laneletMap, laneChangeCost);
    bicycleGraph = setUpGermanBicycleGraph(*laneletMap, laneChangeCost);
  }

  void addPoint(double x, double y, double z) {
    pointId++;
    points.insert(std::pair<Id, Point3d>(pointId, Point3d(pointId, x, y, z)));
  }

  void addLine(const Points3d& points) {
    lineId++;
    lines.insert(std::pair<Id, LineString3d>(lineId, LineString3d(lineId, points)));
  }

  void addLaneletVehicle(const LineString3d& left, const LineString3d& right) {
    laneletId++;
    Lanelet ll{laneletId, left, right};
    ll.setAttribute(AttributeName::Subtype, AttributeValueString::Road);
    lanelets.insert(std::make_pair(laneletId, ll));
  }

  void addLaneletPedestrian(const LineString3d& left, const LineString3d& right) {
    laneletId++;
    Lanelet ll{laneletId, left, right};
    ll.setAttribute(AttributeName::Subtype, AttributeValueString::Crosswalk);
    lanelets.insert(std::make_pair(laneletId, ll));
  }

  void addAreaPedestrian(const LineStrings3d& outerBound) {
    Area area(areaId, outerBound);
    area.setAttribute(AttributeName::Subtype, AttributeValueString::Crosswalk);
    areas.emplace(areaId, area);
    areaId++;
  }

  Id pointId{0};
  Id lineId{1000};
  Id laneletId{2000};
  Id areaId{3000};
  std::unordered_map<Id, Lanelet> lanelets;
  std::unordered_map<Id, Point3d> points;
  std::unordered_map<Id, LineString3d> lines;
  std::unordered_map<Id, Area> areas;
  const double laneChangeCost{2.};
  RoutingGraphPtr vehicleGraph, pedestrianGraph, bicycleGraph;
  LaneletMapPtr laneletMap;

 private:
  void initPoints() {
    points.clear();
    addPoint(0., 1., 0.);  // p1
    addPoint(1., 1., 0.);
    addPoint(2., 1.5, 0.);
    addPoint(0., 0., 0.);
    addPoint(1., 0., 0.);
    addPoint(2., 0.5, 0.);
    addPoint(0., 2., 0.);
    addPoint(1., 2., 0.);
    addPoint(2., 2.5, 0.);  // p9
    addPoint(5., 3.5, 0.);
    addPoint(5., 1.5, 0.);  // p11
    addPoint(7., 3.5, 0.);
    addPoint(7., 1.5, 0.);  // p13
    addPoint(10., 5., 0.);
    addPoint(10., 3., 0.);
    addPoint(10., 1., 0.);  // p16
    addPoint(13., 7., 0.);
    addPoint(13., 5., 0.);
    addPoint(13., 3., 0.);
    addPoint(13., 1., 0.);  // p20
    addPoint(15., 7., 0.);
    addPoint(15., 5., 0.);
    addPoint(15., 3., 0.);
    addPoint(15., 1., 0.);  // p24
    addPoint(5., 14., 0.);
    addPoint(5., 12., 0.);
    addPoint(5., 10., 0.);
    addPoint(7., 14., 0.);
    addPoint(7., 12., 0.);
    addPoint(7., 10., 0.);
    addPoint(10., 13., 0.);  // p31
    addPoint(10., 11., 0.);
    addPoint(12., 13., 0.);
    addPoint(12., 11., 0.);
    addPoint(15., 14., 0.);
    addPoint(15., 12., 0.);
    addPoint(15., 10., 0.);
    addPoint(17., 14., 0.);
    addPoint(17., 12., 0.);
    addPoint(17., 10., 0.);  // p40
    addPoint(17., 8., 0.);
    addPoint(19., 12., 0.);
    addPoint(19., 10., 0.);
    addPoint(19., 8., 0.);  // p44
    addPoint(21., 12., 0.);
    addPoint(21., 10., 0.);  // p46
    addPoint(21., 8., 0.);
    addPoint(23., 12., 0.);   // p48
    addPoint(23., 10., 0.);   // p49
    addPoint(23., 8., 0.);    // p50
    addPoint(10.5, 14., 0.);  // p51
    addPoint(11.5, 14., 0.);  // p52
    addPoint(10.5, 10., 0.);  // p53
    addPoint(11.5, 10., 0.);  // p54
    addPoint(3., 5., 3.);     // p55
    addPoint(4., 5., 3.);     // p56
    addPoint(3., -1., 3.);    // p57
    addPoint(4., -1., 3.);    // p58
    addPoint(26., 10., 0.);   // p59
    addPoint(28., 10., 0.);   // p60
    addPoint(30., 10., 0.);   // p61
    addPoint(25., 11., 0.);   // p62
    addPoint(26., 12., 0.);   // p63
    addPoint(28., 12., 0.);   // p64
    addPoint(30., 12., 0.);   // p65
    addPoint(25., 13., 0.);   // p66
    addPoint(28., 14., 0.);   // p67
    addPoint(30., 14., 0.);   // p68
    addPoint(32., 10., 0.);   // p69
    addPoint(33., 9., 0.);    // p70
    addPoint(32., 8., 0.);    // p71
    addPoint(33., 7., 0.);    // p72
    addPoint(28., 8., 0.);    // p73
    addPoint(30., 8., 0.);    // p74
    addPoint(35., 10., 0.);   // p75
    addPoint(35., 8., 0.);    // p76
    addPoint(35., 6., 0.);    // p77
    addPoint(37., 9., 0.);    // p78
    addPoint(37., 7., 0.);    // p79
    addPoint(37., 5., 0.);    // p80
    addPoint(38., 10., 0.);   // p81
    addPoint(38., 8., 0.);    // p82
    addPoint(41., 10., 0.);   // p83
    addPoint(41., 8., 0.);    // p84
    addPoint(41., 6., 0.);    // p85
    addPoint(43., 10., 0.);   // p86
    addPoint(43., 8., 0.);    // p87

    // points around areas
    pointId = 88;
    addPoint(24, 4, 0);  // p89
    addPoint(26, 4, 0);  // p90
    addPoint(24, 2, 0);  // p91
    addPoint(26, 2, 0);  // p92
    addPoint(27, 5, 0);  // p93
    addPoint(28, 5, 0);  // p94
    addPoint(29, 4, 0);  // p95
    addPoint(29, 2, 0);  // p96
    addPoint(28, 1, 0);  // p97
    addPoint(27, 1, 0);  // p98
    addPoint(27, 0, 0);  // p99
    addPoint(28, 0, 0);  // p100
    addPoint(27, 6, 0);  // p101
    addPoint(28, 6, 0);  // p102
    addPoint(31, 4, 0);  // p103
    addPoint(33, 4, 0);  // p104
    addPoint(31, 2, 0);  // p105
    addPoint(33, 2, 0);  // p106
    addPoint(31, 0, 0);  // p107

    // points on the conflicting and circular section
    pointId = 119;
    addPoint(33, 11, 0);  // p120
    addPoint(37, 16, 0);  // p121
    addPoint(37, 14, 0);  // p122
    addPoint(37, 12, 0);  // p123
    addPoint(41, 12, 0);  // p124
    addPoint(32, 16, 0);  // p125
    addPoint(34, 16, 0);  // p126
    addPoint(28, 18, 0);  // p127
    addPoint(28, 20, 0);  // p128
    addPoint(21, 14, 0);  // p129
    addPoint(23, 14, 0);  // p130
    addPoint(19, 14, 0);  // p131
  }

  void initLineStrings() {
    lines.clear();
    addLine(Points3d{points.at(1), points.at(2)});  // l1001
    lines.at(1001).setAttribute(AttributeNamesString::LaneChange, true);
    addLine(Points3d{points.at(4), points.at(5)});
    addLine(Points3d{points.at(2), points.at(3)});
    addLine(Points3d{points.at(5), points.at(6)});
    addLine(Points3d{points.at(7), points.at(8)});
    addLine(Points3d{points.at(8), points.at(9)});  // ls1006
    addLine(Points3d{points.at(9), points.at(10)});
    addLine(Points3d{points.at(3), points.at(11)});
    lines.at(1008).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(3), points.at(10)});
    lines.at(1009).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(6), points.at(11)});  // ls1010
    addLine(Points3d{points.at(10), points.at(12)});
    addLine(Points3d{points.at(11), points.at(13)});  // ls1012
    addLine(Points3d{points.at(12), points.at(14)});
    addLine(Points3d{points.at(12), points.at(15)});
    addLine(Points3d{points.at(13), points.at(15)});
    lines.at(1015).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(13), points.at(16)});  // ls1016
    addLine(Points3d{points.at(14), points.at(17)});
    addLine(Points3d{points.at(14), points.at(18)});
    addLine(Points3d{points.at(15), points.at(18)});
    lines.at(1019).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(15), points.at(19)});
    lines.at(1020).setAttribute(AttributeNamesString::LaneChange, true);
    addLine(Points3d{points.at(16), points.at(20)});  // ls1021
    addLine(Points3d{points.at(17), points.at(21)});
    addLine(Points3d{points.at(18), points.at(22)});
    lines.at(1023).setAttribute(AttributeNamesString::LaneChange, true);
    addLine(Points3d{points.at(19), points.at(23)});
    lines.at(1024).setAttribute(AttributeNamesString::LaneChange, true);
    addLine(Points3d{points.at(20), points.at(24)});
    addLine(Points3d{points.at(25), points.at(28)});  // ls1026
    addLine(Points3d{points.at(26), points.at(29)});
    addLine(Points3d{points.at(27), points.at(30)});
    addLine(Points3d{points.at(28), points.at(31)});
    addLine(Points3d{points.at(29), points.at(31)});  // ls1030
    lines.at(1030).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(29), points.at(32)});
    lines.at(1031).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(30), points.at(32)});
    addLine(Points3d{points.at(31), points.at(33)});
    addLine(Points3d{points.at(32), points.at(34)});
    addLine(Points3d{points.at(33), points.at(35)});
    addLine(Points3d{points.at(33), points.at(36)});
    lines.at(1036).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(34), points.at(36)});
    lines.at(1037).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(34), points.at(37)});
    addLine(Points3d{points.at(35), points.at(38)});
    addLine(Points3d{points.at(36), points.at(39)});
    addLine(Points3d{points.at(37), points.at(40)});  // ls1041
    addLine(Points3d{points.at(39), points.at(42)});
    addLine(Points3d{points.at(40), points.at(43)});
    lines.at(1043).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1043).setAttribute(AttributeName::Subtype, AttributeValueString::DashedSolid);
    addLine(Points3d{points.at(41), points.at(44)});  // ls1044
    addLine(Points3d{points.at(42), points.at(45)});
    addLine(Points3d{points.at(43), points.at(46)});  // ls1046
    addLine(Points3d{points.at(44), points.at(47)});
    addLine(Points3d{points.at(45), points.at(48)});
    addLine(Points3d{points.at(46), points.at(49)});
    lines.at(1049).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1049).setAttribute(AttributeName::Subtype, AttributeValueString::SolidDashed);
    addLine(Points3d{points.at(47), points.at(50)});  // ls1050
    addLine(Points3d{points.at(51), points.at(53)});  // ls1051
    addLine(Points3d{points.at(52), points.at(54)});  // ls1052
    addLine(Points3d{points.at(55), points.at(57)});  // ls1053
    addLine(Points3d{points.at(56), points.at(58)});  // ls1054
    addLine(Points3d{points.at(49), points.at(59)});  // ls1055
    addLine(Points3d{points.at(59), points.at(60)});  // ls1056
    addLine(Points3d{points.at(60), points.at(61)});  // ls1057
    lines.at(1057).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1057).setAttribute(AttributeName::Subtype, AttributeValueString::Solid);
    addLine(Points3d{points.at(49), points.at(62)});  // ls1058
    lines.at(1058).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(62), points.at(64)});  // ls1059
    lines.at(1059).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(48), points.at(63)});  // ls1060
    lines.at(1060).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(63), points.at(64)});  // ls1061
    lines.at(1061).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(64), points.at(65)});  // ls1062
    lines.at(1062).setAttribute(AttributeNamesString::LaneChange, true);
    addLine(Points3d{points.at(48), points.at(66)});  // ls1063
    addLine(Points3d{points.at(66), points.at(67)});  // ls1064
    addLine(Points3d{points.at(67), points.at(68)});  // ls1065
    addLine(Points3d{points.at(61), points.at(69)});  // ls1066
    addLine(Points3d{points.at(61), points.at(70)});  // ls1067
    addLine(Points3d{points.at(74), points.at(71)});  // ls1068
    addLine(Points3d{points.at(74), points.at(72)});  // ls1069
    addLine(Points3d{points.at(73), points.at(74)});  // ls1070
    addLine(Points3d{points.at(69), points.at(75)});  // ls1071
    addLine(Points3d{points.at(70), points.at(76)});  // ls1072
    addLine(Points3d{points.at(71), points.at(76)});  // ls1073
    addLine(Points3d{points.at(72), points.at(77)});  // ls1074
    addLine(Points3d{points.at(75), points.at(81)});  // ls1075
    addLine(Points3d{points.at(76), points.at(78)});  // ls1076
    addLine(Points3d{points.at(76), points.at(82)});  // ls1077
    addLine(Points3d{points.at(77), points.at(79)});  // ls1078
    addLine(Points3d{points.at(81), points.at(83)});  // ls1079
    addLine(Points3d{points.at(78), points.at(83)});  // ls1080
    addLine(Points3d{points.at(82), points.at(84)});  // ls1081
    addLine(Points3d{points.at(79), points.at(84)});  // ls1082
    lines.at(1082).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1082).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(80), points.at(85)});  // ls1083
    addLine(Points3d{points.at(83), points.at(86)});  // ls1084
    addLine(Points3d{points.at(84), points.at(87)});  // ls1085

    // linestrings around areas
    addLine({points.at(89), points.at(90)});  // ls1086
    addLine({points.at(91), points.at(92)});  // ls1087
    addLine({points.at(90), points.at(93)});  // ls1088
    addLine({points.at(93), points.at(94)});  // ls1089
    addLine({points.at(94), points.at(95)});  // ls1090
    addLine({points.at(95), points.at(96)});  // ls1091
    lines.at(1091).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine({points.at(96), points.at(97)});  // ls1092
    lines.at(1092).setAttribute(AttributeName::Type, AttributeValueString::Curbstone);
    lines.at(1092).setAttribute(AttributeName::Subtype, AttributeValueString::Low);
    addLine({points.at(97), points.at(98)});  // ls1093
    lines.at(1093).setAttribute(AttributeName::Type, AttributeValueString::Curbstone);
    lines.at(1093).setAttribute(AttributeName::Subtype, AttributeValueString::Low);
    addLine({points.at(98), points.at(92)});   // ls1094
    addLine({points.at(95), points.at(103)});  // ls1095
    addLine({points.at(96), points.at(105)});  // ls1096
    lines.at(1096).setAttribute(AttributeName::Type, AttributeValueString::Wall);
    addLine({points.at(103), points.at(105)});  // ls1097
    lines.at(1097).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine({points.at(103), points.at(104)});  // ls1098
    addLine({points.at(105), points.at(106)});  // ls1099
    addLine({points.at(99), points.at(100)});   // ls1100
    addLine({points.at(101), points.at(102)});  // ls1101
    addLine({points.at(92), points.at(90)});    // ls1102
    lines.at(1102).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine({points.at(97), points.at(107)});   // ls1103
    addLine({points.at(107), points.at(105)});  // ls1104

    // lines on the conflicting and circular section
    lineId = 1199;
    addLine({points.at(61), points.at(120)});   // ls1200
    addLine({points.at(74), points.at(70)});    // ls1201
    addLine({points.at(120), points.at(122)});  // ls1202
    addLine({points.at(70), points.at(123)});   // ls1203
    addLine({points.at(121), points.at(124)});  // ls1204
    addLine({points.at(122), points.at(83)});   // ls1205
    addLine({points.at(123), points.at(84)});   // ls1206
    addLine({points.at(68), points.at(125)});   // ls1207
    addLine({points.at(65), points.at(126)});   // ls1208
    addLine({points.at(125), points.at(127)});  // ls1209
    addLine({points.at(126), points.at(128)});  // ls1210
    addLine({points.at(127), points.at(130)});  // ls1211
    addLine({points.at(128), points.at(129)});  // ls1212
    addLine({points.at(130), points.at(48)});   // ls1213
    addLine({points.at(129), points.at(49)});   // ls1214
    addLine({points.at(129), points.at(131)});  // ls1215
    addLine({points.at(130), points.at(42)});   // ls1216
    lines.at(1205).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1205).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
  }
  void initLanelets() {
    lanelets.clear();
    addLaneletVehicle(lines.at(1001), lines.at(1002));  // ll2001
    addLaneletVehicle(lines.at(1003), lines.at(1004));
    addLaneletVehicle(lines.at(1005), lines.at(1001));
    addLaneletVehicle(lines.at(1006), lines.at(1003));  // ll2004
    addLaneletVehicle(lines.at(1007), lines.at(1008));
    addLaneletVehicle(lines.at(1009), lines.at(1010));  // ll2006
    addLaneletVehicle(lines.at(1011), lines.at(1012));  // ll2007
    addLaneletVehicle(lines.at(1013), lines.at(1015));  // ll2008
    addLaneletVehicle(lines.at(1014), lines.at(1016));  // ll2009
    addLaneletVehicle(lines.at(1017), lines.at(1019));  // ll2010
    addLaneletVehicle(lines.at(1018), lines.at(1020));  // ll2011
    addLaneletVehicle(lines.at(1020), lines.at(1021));  // ll2012
    addLaneletVehicle(lines.at(1022), lines.at(1023));  // ll2013
    addLaneletVehicle(lines.at(1023), lines.at(1024));  // ll2014
    addLaneletVehicle(lines.at(1024), lines.at(1025));  // ll2015
    addLaneletVehicle(lines.at(1027).invert(), lines.at(1026).invert());
    addLaneletVehicle(lines.at(1027), lines.at(1028));
    addLaneletVehicle(lines.at(1031).invert(), lines.at(1029).invert());
    addLaneletVehicle(lines.at(1030), lines.at(1032));
    addLaneletVehicle(lines.at(1033), lines.at(1034));
    lanelets.at(2020).setAttribute(AttributeName::OneWay, false);
    addLaneletVehicle(lines.at(1037).invert(), lines.at(1035).invert());
    addLaneletVehicle(lines.at(1036), lines.at(1038));
    lanelets.at(2022).setAttribute(AttributeNamesString::Subtype, AttributeValueString::Highway);
    addLaneletVehicle(lines.at(1040).invert(), lines.at(1039).invert());
    addLaneletVehicle(lines.at(1040), lines.at(1041));  // ll2024
    addLaneletVehicle(lines.at(1042), lines.at(1043));
    addLaneletVehicle(lines.at(1043), lines.at(1044));
    addLaneletVehicle(lines.at(1045), lines.at(1046));  // ll2027
    addLaneletVehicle(lines.at(1046), lines.at(1047));
    addLaneletVehicle(lines.at(1048), lines.at(1049));
    addLaneletVehicle(lines.at(1049), lines.at(1050));     // ll2030
    addLaneletPedestrian(lines.at(1051), lines.at(1052));  // ll2031
    addLaneletVehicle(lines.at(1053), lines.at(1054));     // ll2032
    addLaneletVehicle(lines.at(1060), lines.at(1055));     // ll2033
    addLaneletVehicle(lines.at(1061), lines.at(1056));     // ll2034
    addLaneletVehicle(lines.at(1062), lines.at(1057));     // ll2035
    addLaneletVehicle(lines.at(1063), lines.at(1058));     // ll2036
    addLaneletVehicle(lines.at(1064), lines.at(1059));     // ll2037
    addLaneletVehicle(lines.at(1065), lines.at(1062));     // ll2038
    addLaneletVehicle(lines.at(1066), lines.at(1068));     // ll2039
    addLaneletVehicle(lines.at(1067), lines.at(1069));     // ll2040
    addLaneletVehicle(lines.at(1057), lines.at(1070));     // ll2041
    addLaneletVehicle(lines.at(1071), lines.at(1073));     // ll2042
    addLaneletVehicle(lines.at(1072), lines.at(1074));     // ll2043
    addLaneletVehicle(lines.at(1075), lines.at(1077));     // ll2044
    addLaneletVehicle(lines.at(1076), lines.at(1078));     // ll2045
    addLaneletVehicle(lines.at(1079), lines.at(1081));     // ll2046
    addLaneletVehicle(lines.at(1080), lines.at(1082));     // ll2047
    addLaneletVehicle(lines.at(1082), lines.at(1083));     // ll2048
    addLaneletVehicle(lines.at(1084), lines.at(1085));     // ll2049

    // area
    addLaneletPedestrian(lines.at(1086), lines.at(1087));  // ll2050
    lanelets.at(2050).setAttribute(AttributeName::OneWay, true);
    addLaneletPedestrian(lines.at(1101), lines.at(1089));           // ll2051
    addLaneletPedestrian(lines.at(1093).invert(), lines.at(1100));  // ll2052
    lanelets.at(2052).setAttribute(AttributeName::OneWay, true);
    addLaneletPedestrian(lines.at(1098), lines.at(1099));  // ll2053

    // lanelets on conflicting section
    laneletId = 2059;
    addLaneletVehicle(lines.at(1200), lines.at(1201));  // ll2060
    addLaneletVehicle(lines.at(1202), lines.at(1203));  // ll2061
    addLaneletVehicle(lines.at(1204), lines.at(1205));  // ll2062
    addLaneletVehicle(lines.at(1205), lines.at(1206));  // ll2063
    addLaneletVehicle(lines.at(1207), lines.at(1208));  // ll2064
    addLaneletVehicle(lines.at(1209), lines.at(1210));  // ll2065
    addLaneletVehicle(lines.at(1211), lines.at(1212));  // ll2066
    addLaneletVehicle(lines.at(1213), lines.at(1214));  // ll2067
    addLaneletVehicle(lines.at(1216), lines.at(1215));  // ll2068
  }

  void initAreas() {
    addAreaPedestrian({lines.at(1102), lines.at(1088), lines.at(1089), lines.at(1090), lines.at(1091), lines.at(1092),
                       lines.at(1093), lines.at(1094)});                                           // ar3000
    addAreaPedestrian({lines.at(1095), lines.at(1097), lines.at(1096), lines.at(1091).invert()});  // ar3001
    //    addAreaPedestrian({lines.at(1096).invert(), lines.at(1092), lines.at(1103), lines.at(1104)});  // ar3002
    addAreaPedestrian(
        {lines.at(1096), lines.at(1104).invert(), lines.at(1103).invert(), lines.at(1092).invert()});  // ar3002
    areas.at(3002).setAttribute(AttributeName::Subtype, AttributeValueString::Walkway);
  }
};

namespace {                            // NOLINT
static RoutingGraphTestData testData;  // NOLINT
}  // namespace

class RoutingGraphTest : public ::testing::Test {
 public:
  const std::unordered_map<Id, Lanelet>& lanelets{testData.lanelets};
  const std::unordered_map<Id, Area>& areas{testData.areas};
  const std::unordered_map<Id, Point3d>& points{testData.points};
  const std::unordered_map<Id, LineString3d>& lines{testData.lines};
  const LaneletMapConstPtr laneletMap{testData.laneletMap};
};

class GermanVehicleGraph : public RoutingGraphTest {
 protected:
  GermanVehicleGraph() { EXPECT_NO_THROW(graph->checkValidity()); }  // NOLINT

 public:
  RoutingGraphConstPtr graph{testData.vehicleGraph};
  uint8_t numCostModules{2};
};

class GermanPedestrianGraph : public RoutingGraphTest {
 protected:
  GermanPedestrianGraph() { EXPECT_NO_THROW(graph->checkValidity()); }  // NOLINT

 public:
  RoutingGraphConstPtr graph{testData.pedestrianGraph};
  uint8_t numCostModules{2};
};

class GermanBicycleGraph : public RoutingGraphTest {
 protected:
  GermanBicycleGraph() { EXPECT_NO_THROW(graph->checkValidity()); }  // NOLINT

 public:
  RoutingGraphConstPtr graph{testData.bicycleGraph};
  uint8_t numCostModules{2};
};

template <typename T>
class AllGraphsTest : public T {};

using AllGraphs = testing::Types<GermanVehicleGraph, GermanPedestrianGraph, GermanBicycleGraph>;
TYPED_TEST_CASE(AllGraphsTest, AllGraphs);
}  // namespace tests
}  // namespace routing
}  // namespace lanelet
