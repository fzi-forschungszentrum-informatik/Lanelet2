#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <boost/geometry.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_map_learning/Types.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace map_learning {

using LaneDataPtr = std::shared_ptr<LaneData>;

struct Edge {
  Edge() = default;
  Edge(Id el1, Id el2) : el1_{el1}, el2_{el2} {}
  Id el1_;
  Id el2_;
};

namespace internal {
struct CompoundElsList {
  CompoundElsList(const Id& start, bool startInverted, LineStringType type)
      : ids{start}, inverted{startInverted}, type{type} {}
  CompoundElsList(const std::vector<Id>& ids, const std::vector<bool>& inverted, LineStringType type)
      : ids{ids}, inverted{inverted}, type{type} {}
  std::vector<Id> ids;
  std::vector<bool> inverted;
  LineStringType type;
};
}  // namespace internal

using Edges = std::map<Id, Edge>;  // key = id from

template <typename T>
std::vector<std::shared_ptr<T>> getValidElements(const std::vector<std::shared_ptr<T>>& vec) {
  std::vector<std::shared_ptr<T>> res;
  std::copy_if(vec.begin(), vec.end(), std::back_inserter(res), [](std::shared_ptr<T> el) { return el->valid(); });
  return res;
}

template <typename T>
std::map<Id, std::shared_ptr<T>> getValidElements(const std::map<Id, std::shared_ptr<T>>& map) {
  std::map<Id, std::shared_ptr<T>> res;
  std::copy_if(map.begin(), map.end(), std::inserter(res, res.end()),
               [](const auto& pair) { return pair.second->valid(); });
  return res;
}

class LaneData {
 public:
  struct TensorFeatureData {
   public:
    const std::vector<MatrixXd>& roadBorders() { return roadBorders_; }
    const std::vector<MatrixXd>& laneDividers() { return laneDividers_; }
    const std::vector<int>& laneDividerTypes() { return laneDividerTypes_; }
    const std::vector<MatrixXd>& compoundRoadBorders() { return compoundRoadBorders_; }
    const std::vector<MatrixXd>& compoundLaneDividers() { return compoundLaneDividers_; }
    const std::vector<int>& compoundLaneDividerTypes() { return compoundLaneDividerTypes_; }
    const std::vector<MatrixXd>& compoundCenterlines() { return compoundCenterlines_; }
    const std::string& uuid() { return uuid_; }
    friend class LaneData;

   private:
    std::vector<MatrixXd> roadBorders_;
    std::vector<MatrixXd> laneDividers_;
    std::vector<int> laneDividerTypes_;
    std::vector<MatrixXd> compoundRoadBorders_;
    std::vector<MatrixXd> compoundLaneDividers_;
    std::vector<int> compoundLaneDividerTypes_;
    std::vector<MatrixXd> compoundCenterlines_;
    std::string uuid_;
  };

  LaneData() noexcept : uuid_{boost::lexical_cast<std::string>(boost::uuids::random_generator()())} {}
  static LaneDataPtr build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  bool processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints);

  const LaneLineStringFeatures& roadBorders() { return roadBorders_; }
  const LaneLineStringFeatures& laneDividers() { return laneDividers_; }

  const CompoundLaneLineStringFeatureList& compoundRoadBorders() { return compoundRoadBorders_; }
  const CompoundLaneLineStringFeatureList& compoundLaneDividers() { return compoundLaneDividers_; }
  const CompoundLaneLineStringFeatureList& compoundCenterlines() { return compoundCenterlines_; }

  LaneLineStringFeatures validRoadBorders() { return getValidElements(roadBorders_); }
  LaneLineStringFeatures validLaneDividers() { return getValidElements(laneDividers_); }

  CompoundLaneLineStringFeatureList validCompoundRoadBorders() { return getValidElements(compoundRoadBorders_); }
  CompoundLaneLineStringFeatureList validCompoundLaneDividers() { return getValidElements(compoundLaneDividers_); }
  CompoundLaneLineStringFeatureList validCompoundCenterlines() { return getValidElements(compoundCenterlines_); }

  const std::map<Id, std::vector<size_t>>& associatedCpdRoadBorderIndices() { return associatedCpdRoadBorderIndices_; }
  const std::map<Id, std::vector<size_t>>& associatedCpdLaneDividerIndices() {
    return associatedCpdLaneDividerIndices_;
  }
  const std::map<Id, std::vector<size_t>>& associatedCpdCenterlineIndices() { return associatedCpdCenterlineIndices_; }

  const LaneletFeatures& laneletFeatures() { return laneletFeatures_; }
  const Edges& edges() { return edges_; }
  const std::string& uuid() { return uuid_; }

  /// The computed data will be buffered, if the underlying features change you need to set ignoreBuffer appropriately
  TensorFeatureData getTensorFeatureData(bool pointsIn2d, bool ignoreBuffer);

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
  void getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
                ConstLanelet start, ConstLanelets initPath = ConstLanelets());

  LineStringType getLineStringTypeFromId(Id id);
  LaneLineStringFeaturePtr getLineStringFeatFromId(Id id, bool inverted);
  std::vector<internal::CompoundElsList> computeCompoundLeftBorders(const ConstLanelets& path);
  std::vector<internal::CompoundElsList> computeCompoundRightBorders(const ConstLanelets& path);
  CompoundLaneLineStringFeaturePtr computeCompoundCenterline(const ConstLanelets& path);

  LaneLineStringFeatures roadBorders_;   // auxilliary features
  LaneLineStringFeatures laneDividers_;  // auxilliary features

  CompoundLaneLineStringFeatureList compoundRoadBorders_;   // auxilliary features
  CompoundLaneLineStringFeatureList compoundLaneDividers_;  // auxilliary features
  CompoundLaneLineStringFeatureList compoundCenterlines_;

  LaneletFeatures laneletFeatures_;

  std::map<Id, std::vector<size_t>>
      associatedCpdRoadBorderIndices_;  // relates lanelet id to the "unfiltered" compound features!
  std::map<Id, std::vector<size_t>>
      associatedCpdLaneDividerIndices_;  // relates lanelet id to the "unfiltered" compound features!
  std::map<Id, std::vector<size_t>>
      associatedCpdCenterlineIndices_;  // relates lanelet id to the "unfiltered" compound features!
  Edges edges_;                         // edge list for centerlines
  std::string uuid_;                    // sample id

  Optional<TensorFeatureData> tfData_;
};

/// TODO: finish TE support
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

}  // namespace map_learning
}  // namespace lanelet