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
                             int nPoints) {
  int32_t nodeFeatureLength;
  if (reprType == LaneletRepresentationType::Boundaries)
    nodeFeatureLength = 2 * nPoints + 4;  // 2 boundary types with 2 possible values (one-hot encoding)
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

Eigen::Vector3d getPolylineReprBoundaries(const ConstLanelet& ll, const ParametrizationType& paramType, int nPoints) {
  Eigen::Vector3d repr(2 * 3 * nPoints);
  std::vector<BasicLineString3d> boundaries{ll.leftBound3d().basicLineString(), ll.rightBound3d().basicLineString()};
  for (size_t i = 0; i < boundaries.size(); i++) {
    repr(Eigen::seq(i * 3 * nPoints, (i + 1) * 3 * nPoints)) = computePolylineRepr(boundaries[i], nPoints);
  }
  return repr;
}

Eigen::Vector3d getPolylineReprCenterline(const ConstLanelet& ll, const ParametrizationType& paramType, int nPoints) {
  Eigen::Vector3d repr(3 * nPoints);
  BasicLineString3d centerline = ll.centerline3d().basicLineString();
  return computePolylineRepr(centerline, nPoints);
}

Eigen::Vector3d getNodeFeatureVec(const ConstLanelet& ll, const LaneletRepresentationType& reprType,
                                  const ParametrizationType& paramType, int nPoints, int nodeFeatLength) {
  Eigen::Vector3d featureVec(nodeFeatLength);
  return featureVec;
}

TensorGraphData getLaneLaneData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                                const ParametrizationType& paramType, int nPoints) {
  const auto& graph = localSubmapGraph->graph_;
  const auto& llVertices = graph->vertexLookup();

  TensorGraphData result;
  int numNodes = llVertices.size();
  int32_t nodeFeatLength = getNodeFeatureLength(reprType, paramType, nPoints);
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

    ConstLaneletOrAreas connectedLLs = localSubmapGraph->getLaneletEdges(*ll);
    for (const auto& connectedLL : connectedLLs) {
      result.a.resize(edgeCount + 1, 2);
      result.a(edgeCount, 0) = key2index[la];
      result.a(edgeCount, 0) = key2index[connectedLL];
      edgeCount++;
    }
  }
  return result;
}

TensorGraphData getLaneTEData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                              const ParametrizationType& paramType, int nPoints) {
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