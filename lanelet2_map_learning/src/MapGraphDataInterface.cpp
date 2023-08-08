#include "lanelet2_map_learning/MapGraphDataInterface.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>;
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

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

void MapGraphDataInterface::setCurrPosAndExtractSubmap(const BasicPoint2d& pt) {
  currPos_ = pt;
  localSubmap_ = extractSubmap(laneletMap_, *currPos_, config_.submapAreaX, config_.submapAreaY);
  localSubmapGraph_ = MapGraph::build(*laneletMap_, *trafficRules_);
}

MapGraphDataInterface::MapGraphDataInterface(LaneletMapConstPtr laneletMap, Configuration config,
                                             Optional<BasicPoint2d> currPos)
    : laneletMap_{laneletMap},
      config_{config},
      currPos_{currPos},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

int32_t getNodeFeatureLength(const LaneletRepresentationType& reprType, const ParametrizationType& paramType,
                             int nPoints) {
  int32_t nodeFeatureLength;
  if (reprType == LaneletRepresentationType::Boundaries)
    nodeFeatureLength = 2 * 3 * nPoints + 2;  // 2 boundary types with 2 possible types
  else if (reprType == LaneletRepresentationType::Centerline)
    nodeFeatureLength = 3 * nPoints + 2;
  else
    throw std::runtime_error("Unknown LaneletRepresentationType!");

  return nodeFeatureLength;
}

Eigen::Vector3d computePolylineRepr(const BasicLineString3d& lstr, int32_t nPoints) {
  Eigen::Vector3d repr(3 * nPoints);
  double length = boost::geometry::length(lstr, boost::geometry::strategy::distance::pythagoras<double>());
  double dist = length / static_cast<double>(nPoints);
  boost::geometry::model::multi_point<BasicPoint3d> bdInterp;
  boost::geometry::line_interpolate(lstr, dist, bdInterp);
  assert(bdInterp.size() == nPoints);
  for (size_t i = 0; i < bdInterp.size(); i++) {
    repr(Eigen::seq(i, i + 2)) = bdInterp[i](Eigen::seq(i, i + 2));
  }
  return repr;
}

Eigen::Vector3d getPolylineRepr(const ConstLanelet& ll, const LaneletRepresentationType& reprType,
                                const ParametrizationType& paramType, int nPoints) {
  if (paramType != ParametrizationType::Polyline) {
    throw std::runtime_error("Only polyline parametrization is implemented so far!");
  }
  if (reprType == LaneletRepresentationType::Centerline) {
    Eigen::Vector3d repr(3 * nPoints);
    BasicLineString3d centerline = ll.centerline3d().basicLineString();
    return computePolylineRepr(centerline, nPoints);
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

inline int bdSubtypeToInt(ConstLineString3d lString) {
  std::string subtype = lString.attribute(AttributeName::Subtype).value();
  if (subtype == AttributeValueString::Dashed)
    return 1;
  else if (subtype == AttributeValueString::Solid)
    return 2;
  else if (subtype == AttributeValueString::SolidSolid)
    return 2;
  else if (subtype == AttributeValueString::SolidDashed)
    return 2;
  else if (subtype == AttributeValueString::DashedSolid)
    return 2;
  else {
    throw std::runtime_error("Unexpected Line String Subtype!");
    return 0;
  }
}

Eigen::Vector3d getNodeFeatureVec(const ConstLanelet& ll, const LaneletRepresentationType& reprType,
                                  const ParametrizationType& paramType, int nPoints, int nodeFeatLength) {
  Eigen::Vector3d featureVec(nodeFeatLength);
  Eigen::Vector3d polylineRepr = getPolylineRepr(ll, reprType, paramType, nPoints);
  featureVec(Eigen::seq(0, polylineRepr.size() - 1)) = polylineRepr;
  featureVec[polylineRepr.size()] = bdSubtypeToInt(ll.leftBound3d());
  featureVec[polylineRepr.size() + 1] = bdSubtypeToInt(ll.rightBound3d());
  return featureVec;
}

TensorGraphDataLaneLane MapGraphDataInterface::getLaneLaneData(MapGraphConstPtr localSubmapGraph) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  TensorGraphDataLaneLane result;
  int numNodes = llVertices.size();
  int32_t nodeFeatLength = getNodeFeatureLength(config_.reprType, config_.paramType, config_.nPoints);
  result.x.resize(numNodes, nodeFeatLength);

  std::unordered_map<Id, int> llId2Index = graph->getllId2Index();

  int32_t edgeCount = 0;
  for (const auto& laWithVertex : llVertices) {
    const auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;

    if (nodeFeatureBuffer_.find(la.id()) != nodeFeatureBuffer_.end()) {
      result.x.row(llId2Index[la.id()]) = nodeFeatureBuffer_[la.id()];
    } else {
      Eigen::Vector3d nodeFeatureVec =
          getNodeFeatureVec(*ll, config_.reprType, config_.paramType, config_.nPoints, nodeFeatLength);
      nodeFeatureBuffer_[la.id()] = nodeFeatureVec;
      result.x.row(llId2Index[la.id()]) = nodeFeatureVec;
    }

    ConstLaneletOrAreas connectedLLs = localSubmapGraph->getLaneletEdges(*ll);
    for (const auto& connectedLL : connectedLLs) {
      result.a.resize(edgeCount + 1, 2);
      result.a(edgeCount, 0) = llId2Index[la.id()];
      result.a(edgeCount, 1) = llId2Index[connectedLL.id()];

      result.e.resize(edgeCount + 1, 1);
      ConstLanelet connectedLLasLL = connectedLL.lanelet().get();
      RelationType edgeType = graph->getEdgeInfo(*ll, connectedLLasLL).get().relation;
      result.e.row(edgeCount).array() = relationToInt(edgeType);
      edgeCount++;
    }
  }
  return result;
}

inline int teTypeToInt(const ConstLineString3d& te) {
  std::string type = te.attribute(AttributeName::Type).value();
  std::string subtype = te.attribute(AttributeName::Subtype).value();
  if (type == AttributeValueString::TrafficLight) {
    return 1;
  } else if (type == AttributeValueString::TrafficSign) {
    return 2;
  } else {
    throw std::runtime_error("Unexpected Traffic Element Type!");
    return 0;
  }
}

Eigen::Vector3d getTEPolylineRepr(const BasicLineString3d& te) {
  Eigen::Vector3d repr(12);
  for (size_t i = 0; i < te.size(); i++) {
    repr(Eigen::seq(i, i + 2)) = te[i](Eigen::seq(i, i + 2));
  }
  return repr;
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

TensorGraphDataLaneTE MapGraphDataInterface::getLaneTEData(MapGraphConstPtr localSubmapGraph,
                                                           LaneletSubmapConstPtr localSubmap,
                                                           std::unordered_map<Id, int>& teId2Index) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  TensorGraphDataLaneTE result;
  int numNodesLane = llVertices.size();
  int32_t nodeFeatLengthLane = getNodeFeatureLength(config_.reprType, config_.paramType, config_.nPoints);
  result.xLane.resize(numNodesLane, nodeFeatLengthLane);

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
      result.xLane.row(llId2Index[la.id()]) = nodeFeatureBuffer_[la.id()];
    } else {
      Eigen::Vector3d nodeFeatureVec =
          getNodeFeatureVec(*ll, config_.reprType, config_.paramType, config_.nPoints, nodeFeatLengthLane);
      nodeFeatureBuffer_[la.id()] = nodeFeatureVec;
      result.xLane.row(llId2Index[la.id()]) = nodeFeatureVec;
    }

    RegulatoryElementConstPtrs regElems = ll->regulatoryElements();
    for (const auto& regElem : regElems) {
      ConstLineStrings3d refs = regElem->getParameters<ConstLineString3d>(RoleName::Refers);
      for (const auto& ref : refs) {
        if (isTe(ref)) {
          result.a.resize(edgeCount + 1, 2);
          result.a(edgeCount, 0) = llId2Index[la.id()];
          result.a(edgeCount, 1) = result.xLane.rows() + teId2Index[ref.id()];

          result.xTE.resize(edgeCount + 1, 13);
          if (teFeatureBuffer_.find(ref.id()) != teFeatureBuffer_.end()) {
            result.xTE.row(llId2Index[ref.id()]) = teFeatureBuffer_[ref.id()];
          } else {
            Eigen::Vector3d teFeatureVec = getTEFeatureVec(ref);
            teFeatureBuffer_[ref.id()] = teFeatureVec;
            result.xTE.row(llId2Index[ref.id()]) = teFeatureVec;
          }

          result.e.resize(edgeCount + 1, 1);
          result.e.row(edgeCount).array() = 1;
          edgeCount++;
        }
      }
    }
  }
  return result;
}

TensorGraphDataLaneLane MapGraphDataInterface::laneLaneTensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }

  return getLaneLaneData(localSubmapGraph_);
}

TensorGraphDataLaneTE MapGraphDataInterface::laneTETensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return TensorGraphDataLaneTE();
}

}  // namespace map_learning
}  // namespace lanelet