#include <gtest/gtest.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <matplot/matplot.h>

#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::map_learning;
using namespace lanelet::map_learning::tests;

TEST_F(MapLearningTest, LaneDataPlot) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  ConstLanelets lls;
  lls.insert(lls.end(), laneletMap->laneletLayer.begin(), laneletMap->laneletLayer.end());
  LaneletSubmapConstPtr laneletSubmap = utils::createConstSubmap(lls, {});

  matplot::hold(matplot::on);
  matplot::xlim({-2, 16});
  matplot::ylim({-13, 2});
  matplot::gcf()->size(1000, 1000);

  // for (const auto& ll : lls) {
  //   BasicLineString2d leftBound = ll.leftBound2d().basicLineString();
  //   BasicLineString2d rightBound = ll.rightBound2d().basicLineString();
  //   BasicLineString2d centerline = ll.centerline2d().basicLineString();
  //   std::vector<double> xl;
  //   std::vector<double> yl;
  //   for (const auto& pt : leftBound) {
  //     xl.push_back(pt.x());
  //     yl.push_back(pt.y());
  //   }
  //   std::vector<double> xr;
  //   std::vector<double> yr;
  //   for (const auto& pt : rightBound) {
  //     xr.push_back(pt.x());
  //     yr.push_back(pt.y());
  //   }
  //   std::vector<double> xc;
  //   std::vector<double> yc;
  //   for (const auto& pt : centerline) {
  //     xc.push_back(pt.x());
  //     yc.push_back(pt.y());
  //   }
  //   matplot::plot(xl, yl, "b")->line_width(3);
  //   matplot::plot(xr, yr, "r")->line_width(3);
  //   matplot::plot(xc, yc, "--gs")->line_width(3).marker_color("g");
  // }

  LaneData laneData = LaneData::build(laneletSubmap, laneletMapGraph);

  std::vector<Eigen::MatrixXd> rawCompoundRoadBorders = getPointsMatrixList(laneData.compoundRoadBorders(), true);
  std::vector<Eigen::MatrixXd> rawCompoundLaneDividers = getPointsMatrixList(laneData.compoundLaneDividers(), true);
  std::vector<Eigen::MatrixXd> rawCompoundCenterlines = getPointsMatrixList(laneData.compoundCenterlines(), true);

  for (const auto& mat : rawCompoundRoadBorders) {
    std::vector<double> x;
    std::vector<double> y;
    x.resize(mat.rows());
    y.resize(mat.rows());
    Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    matplot::plot(x, y, "r")->line_width(3);
  }
  for (const auto& mat : rawCompoundLaneDividers) {
    std::vector<double> x;
    std::vector<double> y;
    x.resize(mat.rows());
    y.resize(mat.rows());
    Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    matplot::plot(x, y, "b")->line_width(3);
  }
  for (const auto& mat : rawCompoundCenterlines) {
    std::vector<double> x;
    std::vector<double> y;
    x.resize(mat.rows());
    y.resize(mat.rows());
    Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    matplot::plot(x, y, "--gs")->line_width(3).marker_color("g");
  }

  matplot::save("map_data_raw.png");
  matplot::cla();
  matplot::hold(matplot::on);
  matplot::xlim({-2, 16});
  matplot::ylim({-13, 2});
  matplot::gcf()->size(1000, 1000);

  bool valid = laneData.processAll(bbox, ParametrizationType::LineString, 10);
  std::vector<Eigen::MatrixXd> compoundRoadBorders = getPointsMatrixList(laneData.compoundRoadBorders(), true);
  std::vector<Eigen::MatrixXd> compoundLaneDividers = getPointsMatrixList(laneData.compoundLaneDividers(), true);
  std::vector<Eigen::MatrixXd> compoundCenterlines = getPointsMatrixList(laneData.compoundCenterlines(), true);

  for (const auto& mat : compoundRoadBorders) {
    std::vector<double> x;
    std::vector<double> y;
    x.resize(mat.rows());
    y.resize(mat.rows());
    Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    matplot::plot(x, y, "r")->line_width(3);
  }
  for (const auto& mat : compoundLaneDividers) {
    std::vector<double> x;
    std::vector<double> y;
    x.resize(mat.rows());
    y.resize(mat.rows());
    Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    matplot::plot(x, y, "b")->line_width(3);
  }
  for (const auto& mat : compoundCenterlines) {
    std::vector<double> x;
    std::vector<double> y;
    x.resize(mat.rows());
    y.resize(mat.rows());
    Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    matplot::plot(x, y, "--gs")->line_width(3).marker_color("g");
  }

  matplot::save("map_data_processed.png");
}