#include "lanelet2_validation/validators/mapping/DuplicatedPoints.h"

#include <lanelet2_core/primitives/Point.h>

#include "lanelet2_validation/ValidatorFactory.h"

namespace lanelet {
namespace validation {
namespace {
RegisterMapValidator<DuplicatedPointsChecker> reg;

template <typename T>
Optional<Id> hasDuplicates(const T& elem) {
  if (elem.size() < 2) {
    return {};
  }
  for (auto itFirst = elem.begin(), itSecond = elem.begin() + 1; itSecond != elem.end(); ++itFirst, ++itSecond) {
    if (*itFirst == *itSecond) {
      return itFirst->id();
    }
  }
  return {};
}
}  // namespace

Issues DuplicatedPointsChecker::operator()(const lanelet::LaneletMap& map) {
  Issues issues;
  for (const auto& ls : map.lineStringLayer) {
    auto duplicates = hasDuplicates(ls);
    if (!!duplicates) {
      issues.emplace_back(Severity::Error, Primitive::LineString, ls.id(),
                          "Linestring contains the point " + std::to_string(*duplicates) +
                              " multiple times in succession. This is not allowed!");
    }
  }
  for (const auto& poly : map.polygonLayer) {
    auto duplicates = hasDuplicates(poly);
    if (!!duplicates) {
      issues.emplace_back(Severity::Error, Primitive::Polygon, poly.id(),
                          "Polygon contains the point " + std::to_string(*duplicates) +
                              " multiple times in succession. This is not allowed!");
    }
  }
  return issues;
}
}  // namespace validation
}  // namespace lanelet
