#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <functional>

#include "lanelet2_routing/Forward.h"

namespace lanelet {
namespace routing {

//! This object carries the required information for the graph neighbourhood search
struct LaneletOrAreaVisitInformation {
  ConstLaneletOrArea laneletOrArea;  //!< The lanelet or area that is currently visited
  ConstLaneletOrArea predecessor;    //!< Its predecessor on the shortest path
  double cost{};                     //!< The accumulated cost from the start along shortest path
  size_t length{};                   //!< Number of elements from start to here along shortest path (including this)
  size_t numLaneChanges{};           //!< Number of lane changes from start to here along shortest path
};

//! This object carries the required information for the graph neighbourhood search
struct LaneletVisitInformation {
  ConstLanelet lanelet;      //!< The lanelet or area that is currently visited
  ConstLanelet predecessor;  //!< Its predecessor on the shortest path
  double cost{};             //!< The accumulated cost from the start along shortest path
  size_t length{};           //!< The number of lanelets from start to here along shortest path (including this)
  size_t numLaneChanges{};   //!< Number of lane changes from start to here along shortest path
};

using LaneletVisitFunction = std::function<bool(const LaneletVisitInformation&)>;
using LaneletOrAreaVisitFunction = std::function<bool(const LaneletOrAreaVisitInformation&)>;

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

}  // namespace routing
}  // namespace lanelet
