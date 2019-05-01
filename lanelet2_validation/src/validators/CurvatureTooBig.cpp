#include "validators/mapping/CurvatureTooBig.h"
#include <iostream>
#include "ValidatorFactory.h"
#include "lanelet2_core/geometry/Point.h"

namespace lanelet {
namespace validation {
namespace {
RegisterMapValidator<CurvatureTooBigChecker> reg1;
}  // namespace

Issues CurvatureTooBigChecker::operator()(const LaneletMap& map) {
  Issues issues;
  for (auto& lanelet : map.laneletLayer) {
    checkCurvature(issues, utils::to2D(lanelet.leftBound()), lanelet.id());
    checkCurvature(issues, utils::to2D(lanelet.rightBound()), lanelet.id());
  }
  return issues;
}

double CurvatureTooBigChecker::computeCurvature(const BasicPoint2d& p1, const BasicPoint2d& p2, const BasicPoint2d& p3) {
  auto dp = 0.5 * (p3 - p1);
  auto ddp = p3 - 2.0 * p2 + p1;
  auto denom = std::pow(dp.x() * dp.x() + dp.y() * dp.y(), 3.0 / 2.0);
  if (std::fabs(denom) < 1e-20) {
    denom = 1e-20;
  }
  return static_cast<double>((ddp.y() * dp.x() - dp.y() * ddp.x()) / denom);
}

void CurvatureTooBigChecker::checkCurvature(Issues& issues, const ConstLineString2d& line, const Id& laneletId) {
  auto lineHyb = utils::toHybrid(line);
  if (lineHyb.size() >= 3) {
    for (size_t i = 1; i < lineHyb.size() - 1; ++i) {
      if (std::fabs(computeCurvature(lineHyb[i - 1], lineHyb[i], lineHyb[i + 1])) > 0.5) {
        issues.emplace_back(Severity::Warning, Primitive::Lanelet, laneletId,
                            "Curvature at point " + std::to_string(line[i].id()) +
                            " is bigger than 0.5. This can confuse algorithms using this map.");
      }
    }
  }
}

}  // namespace validation
}  // namespace lanelet
