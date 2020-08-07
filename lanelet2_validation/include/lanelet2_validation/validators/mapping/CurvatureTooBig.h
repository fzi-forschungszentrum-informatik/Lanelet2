#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

class CurvatureTooBigChecker : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.curvature_too_big"; }

  Issues operator()(const LaneletMap& map) override;
  static void checkCurvature(Issues& issues, const ConstLineString2d& line, const Id& laneletId);
};

}  // namespace validation
}  // namespace lanelet
