#include "lanelet2_validation/validators/mapping/CurvatureTooBig.h"

#include <lanelet2_core/geometry/LineString.h>

#include <iostream>

#include "lanelet2_validation/ValidatorFactory.h"

namespace lanelet {
namespace validation {
namespace {
RegisterMapValidator<CurvatureTooBigChecker> reg1;
}  // namespace

Issues CurvatureTooBigChecker::operator()(const LaneletMap& map) {
  Issues issues;
  for (const auto& lanelet : map.laneletLayer) {
    checkCurvature(issues, utils::to2D(lanelet.leftBound()), lanelet.id());
    checkCurvature(issues, utils::to2D(lanelet.rightBound()), lanelet.id());
  }
  return issues;
}

void CurvatureTooBigChecker::checkCurvature(Issues& issues, const ConstLineString2d& line, const Id& laneletId) {
  auto lineHyb = utils::toHybrid(line);
  if (lineHyb.size() >= 3) {
    for (size_t i = 1; i < lineHyb.size() - 1; ++i) {
      if (std::fabs(geometry::curvature2d(lineHyb[i - 1], lineHyb[i], lineHyb[i + 1])) > 0.5) {
        issues.emplace_back(Severity::Warning, Primitive::Lanelet, laneletId,
                            "Curvature at point " + std::to_string(line[i].id()) +
                                " is bigger than 0.5. This can confuse algorithms using this map.");
      }
    }
  }
}

}  // namespace validation
}  // namespace lanelet
