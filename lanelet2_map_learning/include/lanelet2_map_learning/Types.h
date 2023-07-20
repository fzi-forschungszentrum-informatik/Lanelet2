#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <functional>

#include "lanelet2_map_learning/Forward.h"

namespace lanelet {
namespace map_learning {

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
