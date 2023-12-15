#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_map_learning/Types.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace map_learning {

struct Edge {
  Edge() = default;
  Edge(Id el1, Id el2) : el1_{el1}, el2_{el2} {}
  Id el1_;
  Id el2_;
};

namespace internal {
struct CompoundElsList {
  CompoundElsList(const Id& start, LineStringType type) : ids{start}, type{type} {}
  CompoundElsList(const std::vector<Id>& ids, LineStringType type) : ids{ids}, type{type} {}
  std::vector<Id> ids;
  LineStringType type;
};
}  // namespace internal

using Edges = std::map<Id, Edge>;  // key = id from

class LaneData {
 public:
  LaneData() noexcept {}
  static LaneData build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  bool processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints);

  const LaneLineStringFeatures& roadBorders() { return roadBorders_; }
  const LaneLineStringFeatures& laneDividers() { return laneDividers_; }

  const CompoundLaneLineStringFeatureList& compoundRoadBorders() { return compoundRoadBorders_; }
  const CompoundLaneLineStringFeatureList& compoundLaneDividers() { return compoundLaneDividers_; }
  const CompoundLaneLineStringFeatureList& compoundCenterlines() { return compoundCenterlines_; }

  const std::map<Id, std::vector<size_t>>& associatedCpdRoadBorderIndices() { return associatedCpdRoadBorderIndices_; }
  const std::map<Id, std::vector<size_t>>& associatedCpdLaneDividerIndices() {
    return associatedCpdLaneDividerIndices_;
  }
  const std::map<Id, std::vector<size_t>>& associatedCpdCenterlineIndices() { return associatedCpdCenterlineIndices_; }

  const LaneletFeatures& laneletFeatures() { return laneletFeatures_; }
  const Edges& edges() { return edges_; }

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::map_learning::LaneData& feat,
                                              const unsigned int /*version*/);

 private:
  void initLeftBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initRightBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initLaneletFeatures(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initCompoundFeatures(LaneletSubmapConstPtr& localSubmap,
                            lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void updateAssociatedCpdFeatureIndices();

  LineStringType getLineStringTypeFromId(Id id);
  const LaneLineStringFeature& getLineStringFeatFromId(Id id);
  std::vector<internal::CompoundElsList> computeCompoundLeftBorders(const ConstLanelets& path);
  std::vector<internal::CompoundElsList> computeCompoundRightBorders(const ConstLanelets& path);
  CompoundLaneLineStringFeature computeCompoundCenterline(const ConstLanelets& path);

  LaneLineStringFeatures roadBorders_;   // auxilliary features
  LaneLineStringFeatures laneDividers_;  // auxilliary features

  CompoundLaneLineStringFeatureList compoundRoadBorders_;   // auxilliary features
  CompoundLaneLineStringFeatureList compoundLaneDividers_;  // auxilliary features
  CompoundLaneLineStringFeatureList compoundCenterlines_;

  LaneletFeatures laneletFeatures_;
  std::map<Id, std::vector<size_t>> associatedCpdRoadBorderIndices_;   // related to the "unfiltered" compound features!
  std::map<Id, std::vector<size_t>> associatedCpdLaneDividerIndices_;  // related to the "unfiltered" compound features!
  std::map<Id, std::vector<size_t>> associatedCpdCenterlineIndices_;   // related to the "unfiltered" compound features!
  Edges edges_;                                                        // edge list for centerlines
};

class TEData {
 public:
  TEData() noexcept {}

 private:
  LaneletFeatures laneletFeatures;  // node features lanelets
  TEFeatures teFeatures;            // node features traffic elements

  Edges edgeList_;  // edge list
  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
};

template <typename T>
std::vector<T> getValidElements(const std::vector<T>& vec) {
  std::vector<T> res;
  std::copy_if(vec.begin(), vec.end(), std::back_inserter(res), [](T el) { return el.valid(); });
  return res;
}

template <typename T>
std::map<Id, T> getValidElements(const std::map<Id, T>& map) {
  std::map<Id, T> res;
  std::copy_if(map.begin(), map.end(), std::back_inserter(res), [](const auto& pair) { return pair.second.valid(); });
  return res;
}

}  // namespace map_learning
}  // namespace lanelet