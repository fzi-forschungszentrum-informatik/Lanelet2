#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <utility>
#include "Forward.h"

namespace lanelet {
namespace routing {

//! Represents the relation of a lanelet to another lanelet
struct LaneletRelation {
  ConstLanelet lanelet;       //! the lanelet this relation refers to
  RelationType relationType;  //! the type of relation to that
};
using LaneletRelations = std::vector<LaneletRelation>;

//! Represents the relation of a lanelet to another lanelet
struct RouteElementRelation {
  RouteElement* routeElement{nullptr};  //!< The route element this relation refers to
  RelationType relationType{};          //!< The type of relation to that
  operator LaneletRelation() const;     // NOLINT
};
using RouteElementRelations = std::vector<RouteElementRelation>;

/** @brief The fundamental building block of a route.
 * A RouteElement is a wrapper around one lanelet that is part of a route. It provides relational information to
 * other lanelets within the route. Additionally it provides a list of conflicting passable lanelets within the base
 * laneletMap. */
class RouteElement {
 public:
  using LaneId = uint16_t;

  RouteElement(ConstLanelet ll, LaneId lane) : laneId_{lane}, lanelet_{std::move(ll)} {}

  //! Get the relation to the left lanelet if it exists
  inline const Optional<RouteElementRelation>& left() const noexcept { return left_; }

  //! Get the relation to the right lanelet if it exists
  inline const Optional<RouteElementRelation>& right() const noexcept { return right_; }

  //! @brief Returns the laneID of the route element.
  //! @return LaneID of the route element
  inline LaneId laneId() const noexcept { return laneId_; }

  //! @brief Returns the lanelet of the RouteElement
  inline const ConstLanelet& lanelet() const noexcept { return lanelet_; }

  //! Get relations to the previous lanelets if they exist
  const RouteElementRawPtrs& previous() const noexcept { return previous_; }

  //! Get relations to the following lanelets if they exist
  const RouteElementRawPtrs& following() const noexcept { return following_; }

  //! Get the conflicting lanelets within the route if they exist
  const RouteElementRawPtrs& conflictingInRoute() const noexcept { return conflictingInRoute_; }

  //! Get the conflicting lanelets or areas within the passable lanelets in the base laneletMap if they exist
  const ConstLaneletOrAreas& conflictingInMap() const noexcept { return conflictingInMap_; }

  //! Set the left relation
  inline void setLeft(const RouteElementRelation& relation) {
    assert(relation.relationType == RelationType::Left || relation.relationType == RelationType::AdjacentLeft);
    assert(relation.routeElement != this);
    left_ = relation;
  }

  //! Set the right relation
  inline void setRight(const RouteElementRelation& relation) {
    assert(relation.relationType == RelationType::Right || relation.relationType == RelationType::AdjacentRight);
    assert(relation.routeElement != this);
    right_ = relation;
  }

  //! Adds a following route element
  inline void addFollowing(RouteElement* element) {
    assert(element != this);
    assert(std::find(following_.begin(), following_.end(), element) == following_.end());
    following_.emplace_back(element);
  }

  //! Adds a previous route element
  inline void addPrevious(RouteElement* element) {
    assert(element != this);
    assert(std::find(previous_.begin(), previous_.end(), element) == previous_.end());
    previous_.emplace_back(element);
  }

  //! Adds a conflicting element in the route
  inline void addConflictingInRoute(RouteElement* conf) {
    assert(conf != this);
    conflictingInRoute_.emplace_back(conf);
  }

  //! Adds multiple conflicting elements in the route
  inline void addConflictingInRoute(const RouteElementRawPtrs& conf) {
    conflictingInRoute_.insert(conflictingInRoute_.end(), conf.begin(), conf.end());
  }

  //! Adds a conflicting element that is a passable lanelet in the underlying laneletMap
  inline void addConflictingInMap(const ConstLaneletOrArea& conf) {
    assert(conf != this->lanelet());
    conflictingInMap_.emplace_back(conf);
  }

  //! Adds multiple conflicting elements that are passable lanelets in the underlying laneletMap
  inline void addConflictingInMap(const ConstLaneletOrAreas& conf) {
    conflictingInMap_.reserve(conflictingInMap_.size() + conf.size());
    conflictingInMap_.insert(std::end(conflictingInMap_), std::begin(conf), std::end(conf));
  }

  //! Sets the lane ID
  inline void setLaneId(LaneId id) noexcept { laneId_ = id; }

  //! Returns the ID of the referenced lanelet
  inline Id id() const { return lanelet_.id(); }

 private:
  LaneId laneId_{0};                        ///< Final lane ID in the route
  Optional<RouteElementRelation> left_;     ///< Relation to a left route element if it exists
  Optional<RouteElementRelation> right_;    ///< Relation to a right route element if it exists
  RouteElementRawPtrs following_;           ///< Relation to following route elements if they exists
  RouteElementRawPtrs previous_;            ///< Relation to previous route elements if they exists
  RouteElementRawPtrs conflictingInRoute_;  ///< Relations to conflicting route elements in the route
  ConstLaneletOrAreas conflictingInMap_;    ///< Conflicting lanelets in the underlying laneletMap

  // NOLINTNEXTLINE
  const ConstLanelet lanelet_;  ///< The lanelet this route element wraps
};

inline RouteElementRelation::operator LaneletRelation() const {
  return LaneletRelation{routeElement->lanelet(), relationType};
}
inline bool operator==(const LaneletRelation& lhs, const LaneletRelation& rhs) {
  return lhs.lanelet == rhs.lanelet && lhs.relationType == rhs.relationType;
}
inline bool operator!=(const LaneletRelation& rhs, const LaneletRelation& lhs) { return !(rhs == lhs); }
inline bool operator==(const RouteElementRelation& rhs, const RouteElementRelation& lhs) {
  return rhs.routeElement == lhs.routeElement && rhs.relationType == lhs.relationType;
}
inline bool operator!=(const RouteElementRelation& rhs, const RouteElementRelation& lhs) { return !(rhs == lhs); }
}  // namespace routing
}  // namespace lanelet
