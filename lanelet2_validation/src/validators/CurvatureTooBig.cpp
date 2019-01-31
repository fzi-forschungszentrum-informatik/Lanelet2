#include "validators/mapping/CurvatureTooBig.h"
#include <iostream>
#include "ValidatorFactory.h"
#include "lanelet2_core/geometry/Point.h"

namespace lanelet {
namespace validation {
namespace {
RegisterMapValidator<CurvatureTooBigChecker> reg1;
}  // namespace

Issues CurvatureTooBigChecker::operator()(const lanelet::LaneletMap& map) {
  Issues issues;
  for (auto& lanelet : map.laneletLayer) {
    auto left_bound_2d = utils::to2D(lanelet.leftBound());
    auto right_bound_2d = utils::to2D(lanelet.rightBound());
    auto left_bound_2d_hyb = utils::toHybrid(left_bound_2d);
    auto right_bound_2d_hyb = utils::toHybrid(right_bound_2d);
    if (left_bound_2d_hyb.size() >= 3) {
      for (size_t i = 1; i < left_bound_2d_hyb.size() - 1; ++i) {
        if (std::fabs(computeCurvature(left_bound_2d_hyb[i - 1], left_bound_2d_hyb[i], left_bound_2d_hyb[i + 1])) >
            0.5) {
          issues.emplace_back(Severity::Warning, Primitive::Lanelet, lanelet.id(),
                              "Curvature at point " + std::to_string(left_bound_2d[i].id()) +
                              " is bigger than 0.5. This can confuse algorithms using this map.");
        }
      }
    }
    if (right_bound_2d_hyb.size() >= 3) {
      for (size_t i = 1; i < right_bound_2d_hyb.size() - 1; ++i) {
        if (std::fabs(computeCurvature(right_bound_2d_hyb[i - 1], right_bound_2d_hyb[i], right_bound_2d_hyb[i + 1])) >
            0.5) {
          issues.emplace_back(Severity::Warning, Primitive::Lanelet, lanelet.id(),
                              "Curvature at point " + std::to_string(right_bound_2d[i].id()) +
                              " is bigger than 0.5. This can confuse algorithms using this map.");
        }
      }
    }
  }
  return issues;
}

double CurvatureTooBigChecker::computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2,
                                                const lanelet::BasicPoint2d& p3) {
  auto dp = 0.5 * (p3 - p1);
  auto ddp = p3 - 2.0 * p2 + p1;
  auto denom = pow(dp.x() * dp.x() + dp.y() * dp.y(), 3.0 / 2.0);
  if (std::fabs(denom) < 1e-20) {
    denom = 1e-20;
  }
  return static_cast<double>((ddp.y() * dp.x() - dp.y() * ddp.x()) / denom);
}

}  // namespace validation
}  // namespace lanelet
