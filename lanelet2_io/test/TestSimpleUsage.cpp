#include "gtest/gtest.h"

#include "Io.h"

TEST(lanelet2_io, exampleUsage) {  // NOLINT
  using namespace lanelet;
  std::string filenameIn = "test_data/mapping_example.osm";
  LaneletMapPtr laneletMap = lanelet::load(filenameIn);

  std::string filenameOut = "/tmp/mapping_example.osm";
  lanelet::write(filenameOut, *laneletMap);
}
