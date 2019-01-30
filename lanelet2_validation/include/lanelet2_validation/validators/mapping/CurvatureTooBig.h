#include <lanelet2_core/LaneletMap.h>
#include "BasicValidator.h"

namespace lanelet {
namespace validation {

//! This check checks the curvature of the left and right bound of all lanelets in map.
//! This is a common mapping error when users fail to click a point in josm
//! and instead add a new one.

class CurvatureTooBigChecker : public MapValidator {
public:
    constexpr static const char* name() { return "mapping.curvature_too_big"; }

    Issues operator()(const LaneletMap& map) override;
};

}  // namespace validation
}  // namespace lanelet