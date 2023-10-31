#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/Types.h"

namespace lanelet {
namespace map_learning {

struct PolylineFeature {
  BasicLineString3d originalFeature_;
  BasicLineString3d processedFeature_;
  Id mapID_;
  bool wasCut_{false};
  int subtype_;

  ParametrizationType paramType;

  PolylineFeature() {}
  PolylineFeature(const BasicLineString3d& feature, Id mapID) : originalFeature_{feature}, mapID_{mapID} {}

  void processPolyline(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints);
  Eigen::Vector3d computeFeatureVector();
};

struct LaneletFeature {
  PolylineFeature leftBoundary_;
  PolylineFeature rightBoundary_;
  PolylineFeature centerline_;
  Id mapID_;
  bool wasCut_{false};
  int bdTypeLeft_;
  int bdTypeRight_;

  LaneletFeature() {}
  LaneletFeature(const PolylineFeature& leftBoundary, const PolylineFeature& rightBoundary,
                 const PolylineFeature& centerline, Id mapID);
  LaneletFeature(const ConstLanelet& ll);

  void processLanelet(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints);
  Eigen::Vector3d computeFeatureVector(const LaneletRepresentationType& reprType);
};

using PolylineFeatures = std::vector<PolylineFeature>;
using LaneletFeatures = std::vector<LaneletFeature>;

struct LaneData {
  LaneData() noexcept {}
  PolylineFeatures roadBorders;   // auxilliary features
  PolylineFeatures laneDividers;  // auxilliary features

  LaneletFeatures laneletFeatures;  // node features
  Eigen::MatrixX2i edgeList;        // adjacency matrix (sparse)
  Eigen::MatrixXd edgeFeatures;     // edge features
};

struct TEData {
  TEData() noexcept {}
  LaneletFeatures laneletFeatures;  // node features lanelets
  PolylineFeatures teFeatures;      // node features traffic elements

  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
  Eigen::MatrixX2i edgeList;

  Eigen::MatrixXd edgeFeatures;  // edge features
};

}  // namespace map_learning
}  // namespace lanelet