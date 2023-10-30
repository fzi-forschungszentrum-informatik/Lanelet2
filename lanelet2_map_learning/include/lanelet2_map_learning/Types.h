#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <functional>

#include "lanelet2_map_learning/Forward.h"

namespace lanelet {
namespace map_learning {

enum class LaneletRepresentationType { Centerline, Boundaries };
enum class ParametrizationType { Bezier, BezierEndpointFixed, Polyline };

struct PolylineFeature {
  BasicLineString3d originalFeature_;
  BasicLineString3d processedFeature_;
  Id mapID_;
  bool wasCut_{false};
  int subtype_;

  PolylineFeature(BasicLineString3d feature, Id mapID) : originalFeature_{feature} {}

  void computePolyline(int32_t nPoints);
  Eigen::Vector3d computeFeatureVector
};

using PolylineFeatures = std::vector<PolylineFeature>;

struct TensorLaneData {
  TensorLaneData() noexcept {}
  MapFeatures roadBorders;   // auxilliary features
  MapFeatures laneDividers;  // auxilliary features

  MapFeatures laneletFeatures;   // node features
  Eigen::MatrixX2i edgeList;     // adjacency matrix (sparse)
  Eigen::MatrixXd edgeFeatures;  // edge features
};

struct TensorTEData {
  TensorTEData() noexcept {}
  MapFeatures laneletFeatures;  // node features lanelets
  MapFeatures teFeatures;       // node features traffic elements

  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
  Eigen::MatrixX2i edgeList;

  Eigen::MatrixXd edgeFeatures;  // edge features
};

//! Represents the relation of a lanelet to another lanelet
struct LaneletRelation {
  ConstLanelet lanelet;       //!< the lanelet this relation refers to
  RelationType relationType;  //!< the type of relation to that
};
inline bool operator==(const LaneletRelation& lhs, const LaneletRelation& rhs) {
  return lhs.lanelet == rhs.lanelet && lhs.relationType == rhs.relationType;
}
inline bool operator!=(const LaneletRelation& rhs, const LaneletRelation& lhs) { return !(rhs == lhs); }

using LaneletRelations = std::vector<LaneletRelation>;

}  // namespace map_learning
}  // namespace lanelet
