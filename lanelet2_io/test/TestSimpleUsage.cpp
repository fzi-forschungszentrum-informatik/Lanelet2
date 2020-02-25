#include "gtest/gtest.h"

#include "Io.h"

TEST(lanelet2_io, exampleUsage) {  // NOLINT
  using namespace lanelet;
  Origin origin({49, 8.4, 0});
  std::string filenameIn = "../../lanelet2_maps/res/mapping_example.osm";
  LaneletMapPtr laneletMap = lanelet::load(filenameIn, origin);

  std::string filenameOut = std::string(std::tmpnam(nullptr)) + ".osm";  // NOLINT
  lanelet::write(filenameOut, *laneletMap, origin);
  LaneletMapPtr laneletMapAgain = lanelet::load(filenameOut, origin);
}
