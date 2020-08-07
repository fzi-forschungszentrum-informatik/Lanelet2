#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

#include "lanelet2_validation/Cli.h"
#include "lanelet2_validation/Validation.h"

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
  EXPECT_LT(0ul, report.warnings.size());
  EXPECT_EQ(0ul, report.errors.size());
}

TEST(Validator, curvatureTooBig) {  // NOLINT
  std::string exampleMapPath = "../../lanelet2_maps/res/mapping_example.osm";
  using namespace lanelet;
  projection::UtmProjector projector(Origin({49, 8.4}));
  LaneletMapPtr map = load(exampleMapPath, projector);
  lanelet::validation::ValidationConfig config;
  config.checksFilter = "mapping.curvature_too_big";
  auto validators = lanelet::validation::availabeChecks(config.checksFilter);
  auto issues = lanelet::validation::validateMap(*map, config);
  auto report = lanelet::validation::buildReport(issues);
  EXPECT_EQ(0ul, report.warnings.size());
  EXPECT_EQ(0ul, report.errors.size());
}

TEST(Validator, duplicatedPoints) {  // NOLINT
  lanelet::Point3d p{10, 0, 0, 0};
  lanelet::LineString3d ls{5, {p, p}};
  auto map = lanelet::utils::createMap({ls});
  lanelet::validation::ValidationConfig config;
  config.checksFilter = "mapping.duplicated_points";
  auto issues = lanelet::validation::validateMap(*map, config);
  auto report = lanelet::validation::buildReport(issues);
  EXPECT_EQ(0ul, report.warnings.size());
  EXPECT_LT(0ul, report.errors.size());
}

TEST(Validator, invalidMap) {  // NOLINT
  lanelet::validation::ValidationConfig config;
  auto issues = lanelet::validation::validateMap("/totally/nonexistent/fantasy/path", config);
  auto report = lanelet::validation::buildReport(issues);
  EXPECT_EQ(0ul, report.warnings.size());
  EXPECT_LT(0ul, report.errors.size());
}
