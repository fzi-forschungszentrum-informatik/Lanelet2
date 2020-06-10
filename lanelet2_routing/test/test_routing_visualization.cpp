#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include "lanelet2_routing/Exceptions.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing;
using namespace lanelet::routing::tests;
namespace fs = boost::filesystem;

class Tempfile {
 public:
  Tempfile() {
    char path[] = {"/tmp/lanelet2_unittest.XXXXXX"};
    auto* res = mkdtemp(path);
    if (res == nullptr) {
      throw lanelet::LaneletError("Failed to crate temporary directory");
    }
    path_ = path;
  }
  Tempfile(const Tempfile&) = delete;
  Tempfile(Tempfile&&) = delete;
  Tempfile& operator=(const Tempfile&) = delete;
  Tempfile& operator=(Tempfile&&) = delete;
  ~Tempfile() { fs::remove_all(fs::path(path_)); }

  auto operator()(const std::string& str) const noexcept -> std::string { return (fs::path(path_) / str).string(); }

 private:
  std::string path_;
};

static Tempfile tempfile;

TEST_F(GermanVehicleGraph, GraphVizExport) {                            // NOLINT
  EXPECT_NO_THROW(graph->exportGraphViz(tempfile("graphexport.dot")));  // NOLINT
  RelationType excluded{RelationType::Conflicting};
  EXPECT_NO_THROW(graph->exportGraphViz(tempfile("graphexportWithExcluded1.dot"), excluded));  // NOLINT
  excluded |= RelationType::AdjacentLeft;
  excluded |= RelationType::AdjacentRight;
  EXPECT_NO_THROW(graph->exportGraphViz(tempfile("graphexportWithExcluded2.dot"), excluded));  // NOLINT
}

TEST_F(GermanVehicleGraph, GraphVizExportError) {                                           // NOLINT
  EXPECT_THROW(graph->exportGraphViz("", RelationType::None), lanelet::InvalidInputError);  // NOLINT
  EXPECT_THROW(graph->exportGraphViz("/place/that/doesnt/exist"), lanelet::ExportError);    // NOLINT
}

TEST_F(GermanVehicleGraph, GraphMLExport) {                                // NOLINT
  EXPECT_NO_THROW(graph->exportGraphML(tempfile("graphexport.graphml")));  // NOLINT
  RelationType excluded{RelationType::Conflicting};
  EXPECT_NO_THROW(graph->exportGraphML(tempfile("graphexportWithExcluded1.graphml"), excluded));  // NOLINT
  excluded |= RelationType::AdjacentLeft;
  excluded |= RelationType::AdjacentLeft;
  excluded |= RelationType::AdjacentRight;
  EXPECT_NO_THROW(graph->exportGraphML(tempfile("graphexportWithExcluded2.graphml"), excluded));  // NOLINT
}

TEST_F(GermanVehicleGraph, GraphMLExportError) {                                                             // NOLINT
  EXPECT_THROW(graph->exportGraphML("", RelationType::None), lanelet::InvalidInputError);                    // NOLINT
  EXPECT_THROW(graph->exportGraphML("/place/that/doesnt/exist", RelationType::None), lanelet::ExportError);  // NOLINT
}

TEST_F(GermanVehicleGraph, DebugLaneletMap) {  // NOLINT
  LaneletMapPtr map{graph->getDebugLaneletMap()};
  EXPECT_TRUE(map->pointLayer.exists(2007));
  EXPECT_TRUE(map->pointLayer.exists(2020));
  EXPECT_TRUE(map->pointLayer.exists(2032));
  EXPECT_GT(map->lineStringLayer.size(), map->pointLayer.size());
  EXPECT_FALSE(map->pointLayer.exists(2031));
}

TEST_F(GermanPedestrianGraph, DebugLaneletMap) {  // NOLINT
  LaneletMapPtr map{graph->getDebugLaneletMap()};
  EXPECT_FALSE(map->pointLayer.exists(2007));
  EXPECT_FALSE(map->pointLayer.exists(2020));
  EXPECT_FALSE(map->pointLayer.exists(2032));
  EXPECT_TRUE(map->pointLayer.exists(2031));
}

TEST_F(GermanBicycleGraph, DebugLaneletMap) {  // NOLINT
  LaneletMapPtr map{graph->getDebugLaneletMap()};
  EXPECT_TRUE(map->pointLayer.exists(2007));
  EXPECT_TRUE(map->pointLayer.exists(2020));
  EXPECT_TRUE(map->pointLayer.exists(2032));
  EXPECT_FALSE(map->pointLayer.exists(2022));
  EXPECT_FALSE(map->pointLayer.exists(2031));
}

TYPED_TEST(AllGraphsTest, CheckDebugLaneletMap) {
  ASSERT_NO_THROW(this->graph->getDebugLaneletMap());  // NOLINT
  const LaneletMapPtr map{this->graph->getDebugLaneletMap()};
  for (const auto& it : map->pointLayer) {
    EXPECT_NO_THROW(it.attribute("id"));  // NOLINT
  }
  for (const auto& it : map->lineStringLayer) {
    EXPECT_NO_THROW(it.attribute("relation"));  // NOLINT
  }
}
