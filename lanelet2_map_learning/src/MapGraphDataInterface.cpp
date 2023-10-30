#include "lanelet2_map_learning/MapGraphDataInterface.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>;
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

void MapGraphDataInterface::setCurrPosAndExtractSubmap(const BasicPoint2d& pt) {
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

PolylineFeature getPolylineRepr(const ConstLanelet& ll, const LaneletRepresentationType& reprType,
                                const ParametrizationType& paramType, int nPoints) {
  if (paramType != ParametrizationType::Polyline) {
    throw std::runtime_error("Only polyline parametrization is implemented so far!");
  }
  if (reprType == LaneletRepresentationType::Centerline) {
    Eigen::Vector3d repr(3 * nPoints);
    PolylineFeature centerlineFeat(ll.centerline3d().basicLineString(), ll.id());
    centerlineFeat.computePolyline(nPoints);
    return centerlineFeat;
  } else if (reprType == LaneletRepresentationType::Boundaries) {
    Eigen::Vector3d repr(2 * 3 * nPoints);
    std::vector<BasicLineString3d> boundaries{ll.leftBound3d().basicLineString(), ll.rightBound3d().basicLineString()};
    for (size_t i = 0; i < boundaries.size(); i++) {
      repr(Eigen::seq(i * 3 * nPoints, (i + 1) * 3 * nPoints - 1)) = computePolylineRepr(boundaries[i], nPoints);
    }
    return repr;
  } else {
    throw std::runtime_error("Unknown LaneletRepresentationType!");
    return Eigen::Vector3d(3 * nPoints);
  }
}

PolylineFeature getLaneletFeature(const ConstLanelet& ll, const LaneletRepresentationType& reprType,
                                  const ParametrizationType& paramType, int nPoints, int nodeFeatLength) {
  Eigen::Vector3d featureVec(nodeFeatLength);
  Eigen::Vector3d polylineRepr(3 * nPoints);
  PolylineFeature mapFeat = getPolylineRepr(ll, reprType, paramType, nPoints);
  for (size_t i = 0; i < mapFeat.processedFeature_.size(); i++) {
    polylineRepr(Eigen::seq(i, i + 2)) = mapFeat.processedFeature_[i](Eigen::seq(i, i + 2));
  }
  featureVec(Eigen::seq(0, polylineRepr.size() - 1)) = polylineRepr;
  featureVec[polylineRepr.size()] = bdSubtypeToInt(ll.leftBound3d());
  featureVec[polylineRepr.size() + 1] = bdSubtypeToInt(ll.rightBound3d());
  return featureVec;
}

TensorLaneData MapGraphDataInterface::getLaneLaneData(MapGraphConstPtr localSubmapGraph) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  TensorLaneData result;
  int numNodes = llVertices.size();
  int32_t nodeFeatLength = getNodeFeatureLength(config_.reprType, config_.paramType, config_.nPoints);
  result.laneletFeatures.resize(numNodes);

  std::unordered_map<Id, int> llId2Index = graph->getllId2Index();

  int32_t edgeCount = 0;
  for (const auto& laWithVertex : llVertices) {
    const auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;

    if (nodeFeatureBuffer_.find(la.id()) != nodeFeatureBuffer_.end()) {
      result.laneletFeatures[llId2Index[la.id()]] = nodeFeatureBuffer_[la.id()];
    } else {
      MapFeature nodeFeatureVec =
          getMapFeature(*ll, config_.reprType, config_.paramType, config_.nPoints, nodeFeatLength);
      nodeFeatureBuffer_[la.id()] = nodeFeatureVec;
      result.laneletFeatures[llId2Index[la.id()]] = nodeFeatureVec;
    }

    ConstLaneletOrAreas connectedLLs = localSubmapGraph->getLaneletEdges(*ll);
    for (const auto& connectedLL : connectedLLs) {
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

Eigen::Vector3d getTEFeatureVec(const ConstLineString3d& te) {
  if (te.size() != 4) {
    throw std::runtime_error("Number of points in traffic element is not 4!");
  }
  Eigen::Vector3d featureVec(13);  // 4 points with 3 dims + type
  Eigen::Vector3d polylineRepr = getTEPolylineRepr(te.basicLineString());
  featureVec(Eigen::seq(0, polylineRepr.size() - 1)) = polylineRepr;
  featureVec[polylineRepr.size()] = teTypeToInt(te);
  return featureVec;
}

bool isTe(ConstLineString3d ls) {
  std::string type = ls.attribute(AttributeName::Type).value();
  return type == AttributeValueString::TrafficLight || type == AttributeValueString::TrafficSign;
}

TensorTEData MapGraphDataInterface::getLaneTEData(MapGraphConstPtr localSubmapGraph, LaneletSubmapConstPtr localSubmap,
                                                  std::unordered_map<Id, int>& teId2Index) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  TensorTEData result;
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

TensorLaneData MapGraphDataInterface::laneLaneTensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }

  return getLaneLaneData(localSubmapGraph_);
}

TensorTEData MapGraphDataInterface::laneTETensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return TensorTEData();
}

}  // namespace map_learning
}  // namespace lanelet