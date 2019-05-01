#include "validators/mapping/PointsTooClose.h"
#include "ValidatorFactory.h"
#include "lanelet2_core/geometry/Point.h"

namespace lanelet {
namespace validation {
namespace {
RegisterMapValidator<PointsTooCloseChecker> reg;
}  // namespace

Issues PointsTooCloseChecker::operator()(const lanelet::LaneletMap& map) {
  Issues issues;
  for (auto& p : map.pointLayer) {
    auto nearest = map.pointLayer.nearest(utils::to2D(p).basicPoint(), 2);
    if (nearest.size() == 2) {
      auto& next = nearest[0] == p ? nearest[1] : nearest[0];
      if (geometry::distance(p, next) < 0.05) {
        issues.emplace_back(
            Severity::Warning, Primitive::Point, p.id(),
            "Point is very close to point " + std::to_string(next.id()) + ". This can lead to defects in the map");
      }
    }
  }
  return issues;
}

}  // namespace validation
}  // namespace lanelet
