#include "TestSetup.h"
#include "gtest/gtest.h"
#include "lanelet2_io/Io.h"

TEST(lanelet2_io, exampleUsage) {  // NOLINT
  using namespace lanelet;
  Origin origin({49, 8.4, 0});
  std::string filenameIn = "../../lanelet2_maps/res/mapping_example.osm";
  LaneletMapPtr laneletMap = lanelet::load(filenameIn, origin);

  lanelet::test_setup::Tempfile file("file.osm");
  lanelet::write(file.get().string(), *laneletMap, origin);
  LaneletMapPtr laneletMapAgain = lanelet::load(file.get().string(), origin);
}
