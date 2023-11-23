#include "lanelet2_map_learning/MapData.h"

namespace lanelet {
namespace map_learning {

LaneLineStringFeatures extractRoadBorders(MapGraphConstPtr localSubmapGraph) {
  LaneLineStringFeatures roadBorders;
  const auto& internalGraph = localSubmapGraph->internalGraph();
  const auto& llVertices = internalGraph.vertexLookup();

  for (const auto& laWithVertex : llVertices) {
    const auto& la = laWithVertex.first;
    if (!la.isLanelet()) continue;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;
  }
}

LaneLineStringFeatures extractLaneDividers(MapGraphConstPtr localSubmapGraph) {}

CompoundLaneLineStringFeatures extractCompoundRoadBorders(MapGraphConstPtr localSubmapGraph) {}

CompoundLaneLineStringFeatures extractCompoundLaneDividers(MapGraphConstPtr localSubmapGraph) {}

CompoundLaneLineStringFeatures extractCompoundCenterlines(MapGraphConstPtr localSubmapGraph) {}

LaneData::LaneData(MapGraphConstPtr localSubmapGraph) {}

}  // namespace map_learning
}  // namespace lanelet