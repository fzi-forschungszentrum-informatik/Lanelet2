#include "gtest/gtest.h"

#include "Io.h"

TEST(lanelet2_io, exampleUsage) {  // NOLINT
  using namespace lanelet;
  Origin origin({49, 8.4, 0});
  std::string filenameIn = "test_data/mapping_example.osm";
  LaneletMapPtr laneletMap = lanelet::load(filenameIn, origin);

  std::string filenameOut = "/tmp/mapping_example.osm";
  lanelet::write(filenameOut, *laneletMap, origin);
}
