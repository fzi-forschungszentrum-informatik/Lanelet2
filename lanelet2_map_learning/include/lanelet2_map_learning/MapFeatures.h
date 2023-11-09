#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/Types.h"

namespace lanelet {
namespace map_learning {

class MapFeature {
 public:
  const Optional<Id>& mapID() const { return mapID_; }
  const bool& initialized() const { return initialized_; }
  const bool& valid() const { return valid_; }

  virtual Eigen::VectorXd computeFeatureVector(bool /*unused*/) const = 0;
  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t /*unused*/) = 0;

 protected:
  bool initialized_{false};
  bool wasCut_{false};
  bool valid_{true};
  Optional<Id> mapID_;

  MapFeature() {}
  MapFeature(Id mapID) : mapID_{mapID}, initialized_{true} {}
  virtual ~MapFeature() noexcept = default;
};

class LineStringFeature : public MapFeature {
 public:
  const BasicLineString3d& rawFeature() const { return rawFeature_; }

  virtual Eigen::VectorXd computeFeatureVector(bool /*unused*/) const = 0;
  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t /*unused*/) = 0;

 protected:
  BasicLineString3d rawFeature_;

  LineStringFeature() {}
  LineStringFeature(const BasicLineString3d& feature, Id mapID) : MapFeature(mapID), rawFeature_{feature} {}
  virtual ~LineStringFeature() noexcept = default;
};

class LaneLineStringFeature : public LineStringFeature {
 public:
  LaneLineStringFeature() {}
  LaneLineStringFeature(const BasicLineString3d& feature, Id mapID, LineStringType type)
      : LineStringFeature(feature, mapID), type_{type} {}
  virtual ~LaneLineStringFeature() noexcept = default;

  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) override;
  virtual Eigen::VectorXd computeFeatureVector(
      bool onlyPoints = false) const override;  // uses processedFeature_ when available

  const bool& wasCut() const { return wasCut_; }
  const BasicLineString3d& cutFeature() const { return cutFeature_; }
  const BasicLineString3d& cutAndResampledFeature() const { return cutAndResampledFeature_; }
  const LineStringType& type() const { return type_; }

 protected:
  BasicLineString3d cutFeature_;
  BasicLineString3d cutAndResampledFeature_;
  bool wasCut_{false};
  LineStringType type_;
};

class TEFeature : public LineStringFeature {
 public:
  TEFeature() {}
  TEFeature(const BasicLineString3d& feature, Id mapID, TEType type)
      : LineStringFeature(feature, mapID), teType_{type} {}
  virtual ~TEFeature() noexcept = default;

  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t /*unused*/) override;
  Eigen::VectorXd computeFeatureVector(bool onlyPoints = false) const override;

  const TEType& teType() { return teType_; }

 private:
  TEType teType_;
};

class LaneletFeature : public MapFeature {
 public:
  LaneletFeature() {}
  LaneletFeature(const LaneLineStringFeature& leftBoundary, const LaneLineStringFeature& rightBoundary,
                 const LaneLineStringFeature& centerline, Id mapID);
  LaneletFeature(const ConstLanelet& ll);
  virtual ~LaneletFeature() noexcept = default;

  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) override;
  Eigen::VectorXd computeFeatureVector(bool onlyPoints = false) const override;

  void setReprType(LaneletRepresentationType reprType) { reprType_ = reprType; }

  const LaneLineStringFeature& leftBoundary() { return leftBoundary_; }
  const LaneLineStringFeature& rightBoundary() { return rightBoundary_; }
  const LaneLineStringFeature& centerline() { return centerline_; }

 private:
  LaneLineStringFeature leftBoundary_;
  LaneLineStringFeature rightBoundary_;
  LaneLineStringFeature centerline_;
  LaneletRepresentationType reprType_;
};

class CompoundLaneLineStringFeature : public LaneLineStringFeature {
 public:
  CompoundLaneLineStringFeature() {}
  // features for this constructor are required to be given in already sorted order
  CompoundLaneLineStringFeature(const LaneLineStringFeatures& features, LineStringType compoundType);

  virtual ~CompoundLaneLineStringFeature() noexcept = default;

  const LaneLineStringFeatures& features() const { return individualFeatures_; }
  const std::vector<double>& pathLengthsRaw() const { return pathLengthsRaw_; }
  const std::vector<double>& pathLengthsProcessed() const { return pathLengthsProcessed_; }

  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) override;
  Eigen::VectorXd computeFeatureVector(bool onlyPoints = false) const override;

 private:
  LaneLineStringFeatures individualFeatures_;
  std::vector<double> pathLengthsRaw_;
  std::vector<double> pathLengthsProcessed_;
};

using LineStringFeatures = std::vector<LineStringFeature>;
using LaneLineStringFeatures = std::vector<LaneLineStringFeature>;
using CompoundLaneLineStringFeatures = std::vector<CompoundLaneLineStringFeature>;
using TEFeatures = std::vector<TEFeature>;
using LaneletFeatures = std::vector<LaneletFeature>;

}  // namespace map_learning
}  // namespace lanelet