#include <gtest/gtest.h>
#include "Exceptions.h"
#include "RoutingGraph.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing;
using namespace lanelet::routing::tests;

TEST_F(GermanVehicleGraph, GraphVizExport) {  // NOLINT
  const std::string file("/tmp/lanelet2_tests_graphexport.dot");
  const std::string fileWithExcluded1("/tmp/lanelet2_tests_graphexportWithExcluded1.dot");
  const std::string fileWithExcluded2("/tmp/lanelet2_tests_graphexportWithExcluded2.dot");
  EXPECT_NO_THROW(graph->exportGraphViz(file));  // NOLINT
  RelationTypes excluded{RelationType::Conflicting};
  EXPECT_NO_THROW(graph->exportGraphViz(fileWithExcluded1, excluded));  // NOLINT
  excluded.push_back(RelationType::AdjacentLeft);
  excluded.push_back(RelationType::AdjacentRight);
  EXPECT_NO_THROW(graph->exportGraphViz(fileWithExcluded2, excluded));  // NOLINT
}

TEST_F(GermanVehicleGraph, GraphVizExportError) {                                        // NOLINT
  EXPECT_THROW(graph->exportGraphViz("", RelationTypes()), lanelet::InvalidInputError);  // NOLINT
  EXPECT_THROW(graph->exportGraphViz("/bla", RelationTypes()), lanelet::ExportError);    // NOLINT
}

TEST_F(GermanVehicleGraph, GraphMLExport) {  // NOLINT
  const std::string file("/tmp/lanelet2_tests_graphexport.graphml");
  const std::string fileWithExcluded1("/tmp/lanelet2_tests_graphexportWithExcluded1.graphml");
  const std::string fileWithExcluded2("/tmp/lanelet2_tests_graphexportWithExcluded2.graphml");
  EXPECT_NO_THROW(graph->exportGraphML(file));  // NOLINT
  RelationTypes excluded;
  excluded.push_back(RelationType::Conflicting);
  EXPECT_NO_THROW(graph->exportGraphML(fileWithExcluded1, excluded));  // NOLINT
  excluded.push_back(RelationType::AdjacentLeft);
  excluded.push_back(RelationType::AdjacentRight);
  EXPECT_NO_THROW(graph->exportGraphML(fileWithExcluded2, excluded));  // NOLINT
}

TEST_F(GermanVehicleGraph, GraphMLExportError) {                                                          // NOLINT
  EXPECT_THROW(graph->exportGraphML("", RelationTypes()), lanelet::InvalidInputError);                    // NOLINT
  EXPECT_THROW(graph->exportGraphML("/place_that_doesnt_exist", RelationTypes()), lanelet::ExportError);  // NOLINT
}

TEST_F(GermanVehicleGraph, DebugLaneletMap) {  // NOLINT
  LaneletMapPtr map{graph->getDebugLaneletMap()};
  EXPECT_TRUE(map->pointLayer.exists(2007));
  EXPECT_TRUE(map->pointLayer.exists(2020));
  EXPECT_TRUE(map->pointLayer.exists(2032));
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
