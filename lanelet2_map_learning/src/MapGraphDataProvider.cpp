#include "lanelet2_map_learning/MapGraphDataProvider.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>

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
                             int bezierNPoints) {
  int32_t nodeFeatureLength;

  int32_t bdParams;
  if (paramType == ParametrizationType::Bezier)
    bdParams = bezierNPoints;
  else if (paramType == ParametrizationType::BezierEndpointFixed)
    bdParams = bezierNPoints;
  else if (paramType == ParametrizationType::Polyline11Pts)
    bdParams = 11;
  else if (paramType == ParametrizationType::Polyline15Pts)
    bdParams = 15;
  else
    throw std::runtime_error("Unknown LaneletRepresentationType!");

  if (reprType == LaneletRepresentationType::Boundaries)
    nodeFeatureLength = 2 * bdParams + 4;  // 2 boundary types with 2 possible values (one-hot encoding)
  else if (reprType == LaneletRepresentationType::Centerline)
    nodeFeatureLength = 1 * bdParams;
  else
    throw std::runtime_error("Unknown LaneletRepresentationType!");

  return nodeFeatureLength;
}

Eigen::Vector3d computePolylineRepr(const ConstLanelet& ll, int32_t noPts) {
  Eigen::Vector3d repr;
  std::vector<BasicLineString3d> boundaries{ll.leftBound3d().basicLineString(), ll.rightBound3d().basicLineString()};
  for (const auto& bd : boundaries) {
    // Note boost doesnt work here for some reason
    double length = boost::geometry::length(bd, boost::geometry::strategy::distance::pythagoras<double>());
    double dist = length / static_cast<double>(noPts);
    boost::geometry::model::multi_point<BasicPoint3d> bdInterp;
    boost::geometry::line_interpolate(bd, dist, bdInterp);
  }
  return repr;
}

TensorGraphData getLaneLaneData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                                const ParametrizationType& paramType, int bezierNPoints) {
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

    int32_t nodeFeatLength = getNodeFeatureLength(reprType, paramType, bezierNPoints);

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

TensorGraphData getLaneTEData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                              const ParametrizationType& paramType, int bezierNPoints) {
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