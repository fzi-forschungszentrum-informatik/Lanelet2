#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include "Cli.h"
#include "Validation.h"

TEST(TestAllValidators, onExampleMap) {  // NOLINT
  const char* args[] = {"validator",      "../../lanelet2_maps/res/mapping_example.osm",
                        "--participants", "vehicle",
                        "--participants", "pedestrian",
                        "--lat",          "49",
                        "--lon",          "8.4"};
  auto cfg = lanelet::validation::parseCommandLine(sizeof(args) / sizeof(const char*), args);
  EXPECT_EQ(0, lanelet::validation::runFromConfig(cfg));
}

TEST(Validator, pointsTooClose) {  // NOLINT
  using lanelet::Point3d;
  Point3d p1{1, 0, 0, 0};
  Point3d p2{2, 0, 0, 0.01};
  auto map = lanelet::utils::createMap(lanelet::Points3d{p1, p2});
  lanelet::validation::ValidationConfig config;
  config.checksFilter = "mapping.points_too_close";
  auto issues = lanelet::validation::validateMap(*map, config);
  auto report = lanelet::validation::buildReport(issues);
  EXPECT_LT(0ul, report.second.size());
  EXPECT_EQ(0ul, report.first.size());
}

TEST(Validator, invalidMap) {  // NOLINT
  lanelet::validation::ValidationConfig config;
  auto issues = lanelet::validation::validateMap("/totally/nonexistent/fantasy/path", config);
  auto report = lanelet::validation::buildReport(issues);
  EXPECT_EQ(0ul, report.second.size());
  EXPECT_LT(0ul, report.first.size());
}
