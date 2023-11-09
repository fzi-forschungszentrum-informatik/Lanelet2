#include "lanelet2_map_learning/MapGraphDataInterface.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>;
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

void MapGraphDataInterface::setCurrPosAndExtractSubmap(const BasicPoint3d& pt) {
  currPos_ = pt;
  localSubmap_ =
      extractSubmap(laneletMap_, *currPos_, *currYaw_, config_.submapAreaLongitudinal, config_.submapAreaLateral);
  localSubmapGraph_ = MapGraph::build(*laneletMap_, *trafficRules_);
}

MapGraphDataInterface::MapGraphDataInterface(LaneletMapConstPtr laneletMap, Configuration config,
                                             Optional<BasicPoint2d> currPos)
    : laneletMap_{laneletMap},
      config_{config},
      currPos_{currPos},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

LaneData MapGraphDataInterface::getLaneData(MapGraphConstPtr localSubmapGraph) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  ConstLanelets lls;
  Eigen::MatrixX2i edgeList;
  Eigen::MatrixXd edgeFeatures;
  int32_t edgeCount = 0;
  for (const auto& laWithVertex : llVertices) {
    const auto& la = laWithVertex.first;
    if (!la.isLanelet()) continue;

    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;

    if (laneletFeatureBuffer_.find(la.id()) != laneletFeatureBuffer_.end()) {
      lls.push_back(nodeFeatureBuffer_[la.id()]);
    } else {
      MapFeature nodeFeatureVec =
          getMapFeature(*ll, config_.reprType, config_.paramType, config_.nPoints, nodeFeatLength);
      nodeFeatureBuffer_[la.id()] = nodeFeatureVec;
      result.laneletFeatures[llId2Index[la.id()]] = nodeFeatureVec;
    }

    ConstLaneletOrAreas connectedLLs = localSubmapGraph->getLaneletEdges(*ll);
    for (const auto& connectedLL : connectedLLs) {
      if (!connectedLL.isLanelet()) continue;

      result.edgeList.resize(edgeCount + 1, 2);
      result.edgeList(edgeCount, 0) = llId2Index[la.id()];
      result.edgeList(edgeCount, 1) = llId2Index[connectedLL.id()];

      result.edgeList.resize(edgeCount + 1, 1);
      ConstLanelet connectedLLasLL = connectedLL.lanelet().get();
      RelationType edgeType = graph->getEdgeInfo(*ll, connectedLLasLL).get().relation;
      result.edgeList.row(edgeCount).array() = relationToInt(edgeType);
      edgeCount++;
    }
  }
  return result;
}

bool isTe(ConstLineString3d ls) {
  std::string type = ls.attribute(AttributeName::Type).value();
  return type == AttributeValueString::TrafficLight || type == AttributeValueString::TrafficSign;
}

TEData MapGraphDataInterface::getLaneTEData(MapGraphConstPtr localSubmapGraph, LaneletSubmapConstPtr localSubmap,
                                            std::unordered_map<Id, int>& teId2Index) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  TEData result;
  int numNodesLane = llVertices.size();
  int32_t nodeFeatLengthLane = getNodeFeatureLength(config_.reprType, config_.paramType, config_.nPoints);
  result.laneletFeatures.resize(numNodesLane, nodeFeatLengthLane);

  std::unordered_map<Id, int> llId2Index = graph->getllId2Index();

  std::unordered_map<Id, ConstLineString3d> trafficElems;
  for (const auto& ls : localSubmap->lineStringLayer) {
    if (isTe(ls)) {
      trafficElems[ls.id()] = ls;
    }
  }

  teId2Index.clear();
  int i = 0;
  for (const auto& entry : trafficElems) {
    teId2Index[entry.second.id()] = i++;
  }

  int32_t edgeCount = 0;
  for (const auto& laWithVertex : llVertices) {
    const auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;

    if (nodeFeatureBuffer_.find(la.id()) != nodeFeatureBuffer_.end()) {
      result.laneletFeatures.row(llId2Index[la.id()]) = nodeFeatureBuffer_[la.id()];
    } else {
      Eigen::Vector3d nodeFeatureVec =
          getNodeFeatureVec(*ll, config_.reprType, config_.paramType, config_.nPoints, nodeFeatLengthLane);
      nodeFeatureBuffer_[la.id()] = nodeFeatureVec;
      result.laneletFeatures.row(llId2Index[la.id()]) = nodeFeatureVec;
    }

    RegulatoryElementConstPtrs regElems = ll->regulatoryElements();
    for (const auto& regElem : regElems) {
      ConstLineStrings3d refs = regElem->getParameters<ConstLineString3d>(RoleName::Refers);
      for (const auto& ref : refs) {
        if (isTe(ref)) {
          result.edgeList.resize(edgeCount + 1, 2);
          result.edgeList(edgeCount, 0) = llId2Index[la.id()];
          result.edgeList(edgeCount, 1) = result.laneletFeatures.rows() + teId2Index[ref.id()];

          result.teFeatures.resize(edgeCount + 1, 13);
          if (teFeatureBuffer_.find(ref.id()) != teFeatureBuffer_.end()) {
            result.teFeatures.row(llId2Index[ref.id()]) = teFeatureBuffer_[ref.id()];
          } else {
            Eigen::Vector3d teFeatureVec = getTEFeatureVec(ref);
            teFeatureBuffer_[ref.id()] = teFeatureVec;
            result.teFeatures.row(llId2Index[ref.id()]) = teFeatureVec;
          }

          result.edgeFeatures.resize(edgeCount + 1, 1);
          result.edgeFeatures.row(edgeCount).array() = 1;
          edgeCount++;
        }
      }
    }
  }
  return result;
}

LaneData MapGraphDataInterface::laneData() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }

  return getLaneData(localSubmapGraph_);
}

TEData MapGraphDataInterface::teData() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return TEData();
}

}  // namespace map_learning
}  // namespace lanelet