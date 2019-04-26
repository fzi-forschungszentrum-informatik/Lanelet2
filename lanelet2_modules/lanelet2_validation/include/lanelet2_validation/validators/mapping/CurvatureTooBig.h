#include <lanelet2_core/LaneletMap.h>
#include "BasicValidator.h"

namespace lanelet {
namespace validation {

class CurvatureTooBigChecker : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.curvature_too_big"; }

  Issues operator()(const LaneletMap& map) override;
  double computeCurvature(const BasicPoint2d& p1, const BasicPoint2d& p2, const BasicPoint2d& p3);
  void checkCurvature(Issues& issues, const ConstLineString2d& line, const Id& laneletId);
};

}  // namespace validation
}  // namespace lanelet