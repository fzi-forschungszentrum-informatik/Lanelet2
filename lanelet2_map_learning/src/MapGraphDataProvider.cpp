#include "lanelet2_map_learning/MapGraphDataProvider.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace lanelet {
namespace map_learning {

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center, double extentX,
                                    double extentY) {
  BasicPoint2d regionRear = {center.x() - extentX, center.y() - extentY};
  BasicPoint2d regionFront = {center.x() + extentX, center.y() + extentY};
  BoundingBox2d searchRegion{regionRear, regionFront};
  ConstLanelets inRegion = laneletMap->laneletLayer.search(searchRegion);
  return utils::createConstSubmap(inRegion, {});
}

void MapGraphDataProvider::setCurrPosAndExtractSubmap(const BasicPoint2d& pt) {
  currPos_ = pt;
  localSubmap_ = extractSubmap(laneletMap_, *currPos_, config_.submapAreaX, config_.submapAreaY);
  localSubmapGraph_ = MapGraph::build(*laneletMap_, *trafficRules_);
}

MapGraphDataProvider::MapGraphDataProvider(LaneletMapConstPtr laneletMap, Configuration config,
                                           Optional<BasicPoint2d> currPos)
    : laneletMap_{laneletMap},
      config_{config},
      currPos_{currPos},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

TensorGraphData getLaneLaneData(MapGraphConstPtr localSubmapGraph) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  int numNodes = llVertices.size();
  std::unordered_map<ConstLaneletOrArea, int> key2index;
  int i = 0;
  for (const auto& entry : llVertices) {
    key2index[entry.first] = i++;
  }

  TensorGraphData result;
  for (const auto& laWithVertex : llVertices) {
    const auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;
    auto id = la.id();

    ConstLaneletOrAreas connectedLLs = localSubmapGraph->getLaneletEdges(*ll);
    int32_t edgeCount = 1;
    for (const auto& connectedLL : connectedLLs) {
      result.a.resize(edgeCount, 2);
      result.a(edgeCount - 1, 0) = key2index[la];
      result.a(edgeCount - 1, 0) = key2index[connectedLL];
      edgeCount++;
    }
  }
  return result;
}

TensorGraphData getLaneTEData(MapGraphConstPtr localSubmapGraph) { return TensorGraphData(); }

TensorGraphData MapGraphDataProvider::laneLaneTensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }

  return TensorGraphData();
}

TensorGraphData MapGraphDataProvider::laneTETensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return TensorGraphData();
}

}  // namespace map_learning
}  // namespace lanelet