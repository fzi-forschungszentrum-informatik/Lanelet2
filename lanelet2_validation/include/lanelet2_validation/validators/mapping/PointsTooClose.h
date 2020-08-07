#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

//! This check looks for points that are suspiciosly close to each other.
//! This is a common mapping error when users fail to click a point in josm
//! and instead add a new one.
class PointsTooCloseChecker : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.points_too_close"; }

  Issues operator()(const LaneletMap& map) override;
};

}  // namespace validation
}  // namespace lanelet
