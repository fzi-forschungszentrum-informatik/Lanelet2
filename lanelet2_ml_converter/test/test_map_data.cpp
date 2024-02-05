#include <gtest/gtest.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
// #include <matplot/matplot.h>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

TEST_F(MLConverterTest, LaneData) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  ConstLanelets lls;
  lls.insert(lls.end(), laneletMap->laneletLayer.begin(), laneletMap->laneletLayer.end());
  LaneletSubmapConstPtr laneletSubmap = utils::createConstSubmap(lls, {});

  LaneDataPtr laneData = LaneData::build(laneletSubmap, laneletMapGraph);

  bool valid = laneData->processAll(bbox, ParametrizationType::LineString, 20);
  std::vector<Eigen::MatrixXd> compoundRoadBorders = getPointMatrices(laneData->compoundRoadBorders(), true);
  std::vector<Eigen::MatrixXd> compoundLaneDividers = getPointMatrices(laneData->compoundLaneDividers(), true);
  std::vector<Eigen::MatrixXd> compoundCenterlines = getPointMatrices(laneData->compoundCenterlines(), true);

  EXPECT_TRUE(laneData->laneletInstances().find(2007) != laneData->laneletInstances().end());
  EXPECT_EQ(laneData->laneletInstances().find(2007)->second->leftBoundary()->mapID(), 1012);
  EXPECT_TRUE(laneData->roadBorders().find(1001) != laneData->roadBorders().end());

  EXPECT_EQ(compoundRoadBorders.size(), 3);
  EXPECT_EQ(compoundLaneDividers.size(), 8);
  EXPECT_EQ(compoundCenterlines.size(), 4);

  EXPECT_EQ(laneData->associatedCpdRoadBorders(2001).size(), 1);
  CompoundLaneLineStringInstancePtr assoBorder = laneData->associatedCpdRoadBorders(2001).front();
  EXPECT_EQ(assoBorder->features().back()->mapID(), 1000);
  EXPECT_EQ(assoBorder->features().back()->laneletIDs().front(), 2002);

  EXPECT_EQ(laneData->associatedCpdLaneDividers(2004).size(), 2);
  EXPECT_EQ(laneData->associatedCpdLaneDividers(2009).size(), 1);
  CompoundLaneLineStringInstancePtr assoDivider = laneData->associatedCpdLaneDividers(2009).front();
  EXPECT_EQ(assoDivider->features().front()->mapID(), 1015);
  EXPECT_EQ(assoDivider->features().front()->laneletIDs().size(), 2);

  EXPECT_EQ(laneData->associatedCpdCenterlines(2003).size(), 2);
  EXPECT_EQ(laneData->associatedCpdCenterlines(2000).size(), 1);
  CompoundLaneLineStringInstancePtr assoCenterline = laneData->associatedCpdCenterlines(2000).front();
  EXPECT_EQ(assoCenterline->features().front()->mapID(), 2000);
  EXPECT_EQ(assoCenterline->features().front()->laneletIDs().front(), 2000);

  // matplot::hold(matplot::on);
  // matplot::xlim({-2, 16});
  // matplot::ylim({-13, 2});
  // matplot::gcf()->size(1000, 1000);
  //
  // for (const auto& mat : rawCompoundRoadBorders) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "r")->line_width(3);
  // }
  // for (const auto& mat : rawCompoundLaneDividers) {
  //   std::vector<double> x#include <matplot/matplot.h>
  //   std::vector<double> y#include <matplot/matplot.h>
  //   x.resize(mat.rows());#include <matplot/matplot.h>
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "b")->line_width(3);
  // }
  // for (const auto& mat : rawCompoundCenterlines) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "--gs")->line_width(3).marker_color("g");
  // }
  //
  // matplot::save("map_data_raw.png");
  // matplot::cla();
  // matplot::hold(matplot::on);
  // matplot::xlim({-2, 16});
  // matplot::ylim({-13, 2});
  // matplot::gcf()->size(1000, 1000);
  //
  // for (const auto& mat : compoundRoadBorders) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "r")->line_width(3);
  // }
  // for (const auto& mat : compoundLaneDividers) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "b")->line_width(3);
  // }
  // for (const auto& mat : compoundCenterlines) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "--gs")->line_width(3).marker_color("g");
  // }
  // matplot::save("map_data_processed.png");
}