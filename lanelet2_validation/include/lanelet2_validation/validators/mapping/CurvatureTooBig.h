#include <lanelet2_core/LaneletMap.h>
#include "BasicValidator.h"

namespace lanelet {
namespace validation {

class CurvatureTooBigChecker : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.curvature_too_big"; }

  Issues operator()(const LaneletMap& map) override;
  double computeCurvature(const BasicPoint2d&, const BasicPoint2d&, const BasicPoint2d&);
  void checkCurvature(Issues&, const ConstLineString2d&, const Id&);
};

}  // namespace validation
}  // namespace lanelet