#include "LaneletPath.h"
#include <lanelet2_core/geometry/Lanelet.h>

namespace lanelet {
namespace routing {

LaneletSequence LaneletPath::getRemainingLane(LaneletPath::const_iterator laneletPosition) const {
  ConstLanelets lane;
  while (laneletPosition != lanelets_.end()) {
    lane.push_back(*laneletPosition);
    if (laneletPosition + 1 == lanelets_.end() || !geometry::follows(*laneletPosition, *std::next(laneletPosition))) {
      break;
    }
    ++laneletPosition;
  }
  return lane;
}
}  // namespace routing
}  // namespace lanelet
