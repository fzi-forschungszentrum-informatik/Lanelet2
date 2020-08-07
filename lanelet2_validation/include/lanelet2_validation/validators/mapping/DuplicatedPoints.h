#pragma once
#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

//! This check looks for points within linestrings or polygons that appear two times in succession. These are not
//! allowed because they often confuse geometry algorithms.
class DuplicatedPointsChecker : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.duplicated_points"; }

  Issues operator()(const LaneletMap& map) override;
};

}  // namespace validation
}  // namespace lanelet
