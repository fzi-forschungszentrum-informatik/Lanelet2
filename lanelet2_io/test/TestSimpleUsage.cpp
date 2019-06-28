#include "gtest/gtest.h"

#include "boost/filesystem.hpp"
#include "Io.h"

namespace fs = boost::filesystem;

TEST(lanelet2_io, exampleUsage) {  // NOLINT
  using namespace lanelet;
  Origin origin({49, 8.4, 0});
  std::string filenameIn = "../../lanelet2_maps/res/mapping_example.osm";
  LaneletMapPtr laneletMap = lanelet::load(filenameIn, origin);

  std::string filenameOut = std::string(fs::unique_path().string()) + ".osm";  // NOLINT
  lanelet::write(filenameOut, *laneletMap, origin);
  LaneletMapPtr laneletMapAgain = lanelet::load(filenameOut, origin);
}
