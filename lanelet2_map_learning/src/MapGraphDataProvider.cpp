#include "lanelet2_map_learning/MapGraphDataProvider.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
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

int32_t getNodeFeatureLength(const LaneletRepresentationType& reprType, const ParametrizationType& paramType,
                             int nPoints, int noBdTypes) {
  int32_t nodeFeatureLength;
  if (reprType == LaneletRepresentationType::Boundaries)
    nodeFeatureLength =
        2 * nPoints + 2 * noBdTypes;  // 2 boundary types with 2 possible values and unknown types (one-hot encoding)
  else if (reprType == LaneletRepresentationType::Centerline)
    nodeFeatureLength = 1 * nPoints;
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

inline int linestringSubtypeToInt(ConstLineString3d lString) {
  std::string subtype = lString.attribute(AttributeName::Subtype).value();
  if (subtype == AttributeValueString::Dashed)
    return 0;
  else if (subtype == AttributeValueString::Solid)
    return 1;
  else if (subtype == AttributeValueString::SolidSolid)
    return 1;
  else if (subtype == AttributeValueString::SolidDashed)
    return 1;
  else if (subtype == AttributeValueString::DashedSolid)
    return 1;
  else {
    throw std::runtime_error("Unexpected Line String Subtype!");
    return 1;
  }
}

Eigen::Vector3d getNodeFeatureVec(const ConstLanelet& ll, const LaneletRepresentationType& reprType,
                                  const ParametrizationType& paramType, int nPoints, int nodeFeatLength,
                                  int noBdTypes) {
  Eigen::Vector3d featureVec(nodeFeatLength);
  Eigen::Vector3d polylineRepr = getPolylineRepr(ll, reprType, paramType, nPoints);
  featureVec(Eigen::seq(0, polylineRepr.size() - 1)) = polylineRepr;

  Eigen::Vector3d typeFeatureVecLeft(noBdTypes);
  typeFeatureVecLeft.setZero();
  typeFeatureVecLeft[linestringSubtypeToInt(ll.leftBound3d())] = 1;

  Eigen::Vector3d typeFeatureVecRight(noBdTypes);
  typeFeatureVecRight.setZero();
  typeFeatureVecRight[linestringSubtypeToInt(ll.rightBound3d())] = 1;

  featureVec(Eigen::seq(polylineRepr.size(), noBdTypes - 1)) = typeFeatureVecLeft;
  featureVec(Eigen::seq(polylineRepr.size() + noBdTypes, polylineRepr.size() + 2 * noBdTypes - 1)) =
      typeFeatureVecRight;

  return featureVec;
}

TensorGraphData getLaneLaneData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                                const ParametrizationType& paramType, int nPoints, int noRelTypes = 7,
                                int noBdTypes = 3) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  TensorGraphData result;
  int numNodes = llVertices.size();
  int32_t nodeFeatLength = getNodeFeatureLength(reprType, paramType, nPoints, noBdTypes);
  result.x.resize(numNodes, nodeFeatLength);

  std::unordered_map<ConstLaneletOrArea, int> key2index;
  int i = 0;
  for (const auto& entry : llVertices) {
    key2index[entry.first] = i++;
  }

  int32_t edgeCount = 0;
  for (const auto& laWithVertex : llVertices) {
    const auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;
    auto id = la.id();
    result.x.row(key2index[la]) = getNodeFeatureVec(*ll, reprType, paramType, nPoints, nodeFeatLength, noBdTypes);

    ConstLaneletOrAreas connectedLLs = localSubmapGraph->getLaneletEdges(*ll);
    for (const auto& connectedLL : connectedLLs) {
      result.a.resize(edgeCount + 1, 2);
      result.a(edgeCount, 0) = key2index[la];
      result.a(edgeCount, 0) = key2index[connectedLL];

      result.e.resize(edgeCount + 1, noRelTypes);
      Eigen::Vector3d edgeFeatureVec(noRelTypes);
      edgeFeatureVec.setZero();
      ConstLanelet connectedLLasLL = connectedLL.lanelet().get();
      RelationType edgeType = graph->getEdgeInfo(*ll, connectedLLasLL).get().relation;
      edgeFeatureVec[relationToInt(edgeType)] = 1;
      result.e.row(edgeCount) = edgeFeatureVec;
      edgeCount++;
    }
  }
  return result;
}

TensorGraphData getLaneTEData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                              const ParametrizationType& paramType, int nPoints, int noRelTypes = 7,
                              int noBdTypes = 3) {
  return TensorGraphData();
}

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