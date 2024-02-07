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

#include "lanelet2_ml_converter/Forward.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Types.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace ml_converter {

using LaneDataPtr = std::shared_ptr<LaneData>;

struct Edge {
  Edge() = default;
  Edge(Id el1, Id el2, bool isLaneChange) : el1_{el1}, el2_{el2}, isLaneChange_{isLaneChange} {}
  Id el1_;
  Id el2_;
  bool isLaneChange_;
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

using Edges = std::map<Id, std::vector<Edge>>;  // key = id from

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
  struct TensorInstanceData {
   public:
    const std::vector<MatrixXd>& roadBorders() { return roadBorders_; }
    const std::vector<MatrixXd>& laneDividers() { return laneDividers_; }
    const std::vector<int>& laneDividerTypes() { return laneDividerTypes_; }
    const std::vector<MatrixXd>& compoundRoadBorders() { return compoundRoadBorders_; }
    const std::vector<MatrixXd>& compoundLaneDividers() { return compoundLaneDividers_; }
    const std::vector<int>& compoundLaneDividerTypes() { return compoundLaneDividerTypes_; }
    const std::vector<MatrixXd>& compoundCenterlines() { return compoundCenterlines_; }
    const std::string& uuid() { return uuid_; }
    CompoundLaneLineStringInstancePtr pointMatrixCpdRoadBorder(
        size_t index);  // get feature associated to point matrix index
    CompoundLaneLineStringInstancePtr pointMatrixCpdLaneDivider(
        size_t index);  // get feature associated to point matrix index
    CompoundLaneLineStringInstancePtr pointMatrixCpdCenterline(
        size_t index);  // get feature associated to point matrix index
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
    std::map<size_t, CompoundLaneLineStringInstancePtr>
        pointMatrixCpdRoadBorder_;  // relates point matrix index to compound features
    std::map<size_t, CompoundLaneLineStringInstancePtr>
        pointMatrixCpdLaneDivider_;  // relates point matrix index to compound features, tfData_ must be initialized
    std::map<size_t, CompoundLaneLineStringInstancePtr>
        pointMatrixCpdCenterline_;  // relates point matrix index to compound features, tfData_ must be initialized
  };

  LaneData() noexcept : uuid_{boost::lexical_cast<std::string>(boost::uuids::random_generator()())} {}
  static LaneDataPtr build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  bool processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints, double pitch = 0,
                  double roll = 0);

  const LaneLineStringInstances& roadBorders() { return roadBorders_; }
  const LaneLineStringInstances& laneDividers() { return laneDividers_; }

  const CompoundLaneLineStringInstanceList& compoundRoadBorders() { return compoundRoadBorders_; }
  const CompoundLaneLineStringInstanceList& compoundLaneDividers() { return compoundLaneDividers_; }
  const CompoundLaneLineStringInstanceList& compoundCenterlines() { return compoundCenterlines_; }

  LaneLineStringInstances validRoadBorders() { return getValidElements(roadBorders_); }
  LaneLineStringInstances validLaneDividers() { return getValidElements(laneDividers_); }

  CompoundLaneLineStringInstanceList validCompoundRoadBorders() { return getValidElements(compoundRoadBorders_); }
  CompoundLaneLineStringInstanceList validCompoundLaneDividers() { return getValidElements(compoundLaneDividers_); }
  CompoundLaneLineStringInstanceList validCompoundCenterlines() { return getValidElements(compoundCenterlines_); }

  CompoundLaneLineStringInstanceList associatedCpdRoadBorders(
      Id mapId);  // get features associated to map element with id
  CompoundLaneLineStringInstanceList associatedCpdLaneDividers(
      Id mapId);  // get features associated to map element with id
  CompoundLaneLineStringInstanceList associatedCpdCenterlines(
      Id mapId);  // get features associated to map element with id

  const LaneletInstances& laneletInstances() { return laneletInstances_; }
  const Edges& edges() { return edges_; }
  const std::string& uuid() { return uuid_; }

  /// The computed data will be buffered, if the underlying features change you need to set ignoreBuffer appropriately
  /// The data will also only be useful if you called process on the features beforehand (e.g. with processAll())
  TensorInstanceData getTensorInstanceData(bool pointsIn2d, bool ignoreBuffer);

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::ml_converter::LaneData& feat,
                                              const unsigned int /*version*/);

 private:
  void initLeftBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initRightBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initLaneletInstances(LaneletSubmapConstPtr& localSubmap,
                            lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initCompoundInstances(LaneletSubmapConstPtr& localSubmap,
                             lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void updateAssociatedCpdInstanceIndices();
  void getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
                ConstLanelet start, ConstLanelets initPath = ConstLanelets());

  LineStringType getLineStringTypeFromId(Id id);
  LaneLineStringInstancePtr getLineStringFeatFromId(Id id, bool inverted);
  std::vector<internal::CompoundElsList> computeCompoundLeftBorders(const ConstLanelets& path);
  std::vector<internal::CompoundElsList> computeCompoundRightBorders(const ConstLanelets& path);
  CompoundLaneLineStringInstancePtr computeCompoundCenterline(const ConstLanelets& path);

  LaneLineStringInstances roadBorders_;   // auxilliary features
  LaneLineStringInstances laneDividers_;  // auxilliary features

  CompoundLaneLineStringInstanceList compoundRoadBorders_;   // auxilliary features
  CompoundLaneLineStringInstanceList compoundLaneDividers_;  // auxilliary features
  CompoundLaneLineStringInstanceList compoundCenterlines_;

  LaneletInstances laneletInstances_;

  std::map<Id, std::vector<size_t>>
      associatedCpdRoadBorderIndices_;  // relates map element id to the "unfiltered" compound features!
  std::map<Id, std::vector<size_t>>
      associatedCpdLaneDividerIndices_;  // relates map element id to the "unfiltered" compound features!
  std::map<Id, std::vector<size_t>>
      associatedCpdCenterlineIndices_;  // relates map element id to the "unfiltered" compound features!

  Edges edges_;       // edge list for centerlines
  std::string uuid_;  // sample id

  Optional<TensorInstanceData> tfData_;
};

/// TODO: finish TE support
class TEData {
 public:
  TEData() noexcept {}

 private:
  LaneletInstances laneletInstances;  // node features lanelets
  TEInstances teInstances;            // node features traffic elements

  Edges edgeList_;  // edge list
  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
};

}  // namespace ml_converter
}  // namespace lanelet