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

struct LineStringFeature {
  BasicLineString3d rawFeature_;
  Id mapID_;

  LineStringFeature() {}
  LineStringFeature(const BasicLineString3d& feature, Id mapID) : rawFeature_{feature}, mapID_{mapID} {}

  virtual Eigen::VectorXd computeFeatureVector();
};

struct LaneLineStringFeature : LineStringFeature {
  BasicLineString3d processedFeature_;
  bool wasCut_{false};
  LineStringType type_;

  ParametrizationType paramType;

  LaneLineStringFeature() {}
  LaneLineStringFeature(const BasicLineString3d& feature, Id mapID, LineStringType type)
      : LineStringFeature(feature, mapID), type_{type} {}

  void processLineString(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints);
  Eigen::VectorXd computeFeatureVector(bool onlyPoints = false);  // uses processedFeature_ when available
};

struct TEFeature : LineStringFeature {
  TEType teType_;

  TEFeature() {}
  TEFeature(const BasicLineString3d& feature, Id mapID, TEType type)
      : LineStringFeature(feature, mapID), teType_{type} {}

  Eigen::VectorXd computeFeatureVector();
};

struct LaneletFeature {
  LaneLineStringFeature leftBoundary_;
  LaneLineStringFeature rightBoundary_;
  LaneLineStringFeature centerline_;
  Id mapID_;
  bool wasCut_{false};

  LaneletFeature() {}
  LaneletFeature(const LaneLineStringFeature& leftBoundary, const LaneLineStringFeature& rightBoundary,
                 const LaneLineStringFeature& centerline, Id mapID);
  LaneletFeature(const ConstLanelet& ll);

  void processLanelet(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints);
  Eigen::VectorXd computeFeatureVector(const LaneletRepresentationType& reprType, bool onlyPoints = false);
};

struct CompoundLaneLineStringFeature {
  LaneLineStringFeatures features_;
  std::vector<double> pathLengths_;
  LineStringType compoundType_;
  CompoundLaneLineStringFeature() {}
  // features for this constructor are required to be given in sorted order
  CompoundLaneLineStringFeature(const LaneLineStringFeatures& features, LineStringType compoundType);
  virtual Eigen::VectorXd computeFeatureVector();
};

using LineStringFeatures = std::vector<LineStringFeature>;
using LaneLineStringFeatures = std::vector<LaneLineStringFeature>;
using TEFeatures = std::vector<TEFeature>;
using LaneletFeatures = std::vector<LaneletFeature>;

struct LaneData {
  LaneData() noexcept {}
  LaneLineStringFeatures roadBorders;   // auxilliary features
  LaneLineStringFeatures laneDividers;  // auxilliary features

  LaneletFeatures laneletFeatures;  // node features
  Eigen::MatrixX2i edgeList;        // adjacency matrix (sparse)
  Eigen::MatrixXd edgeFeatures;     // edge features
};

struct TEData {
  TEData() noexcept {}
  LaneletFeatures laneletFeatures;  // node features lanelets
  TEFeatures teFeatures;            // node features traffic elements

  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
  Eigen::MatrixX2i edgeList;

  Eigen::MatrixXd edgeFeatures;  // edge features
};

}  // namespace map_learning
}  // namespace lanelet