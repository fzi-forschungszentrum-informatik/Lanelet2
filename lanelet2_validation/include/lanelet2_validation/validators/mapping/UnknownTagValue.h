#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

//! This checks that known tags have known values.
class UnknownTagValue : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.unknown_tag_value"; }

  Issues operator()(const LaneletMap& map) override;
};

}  // namespace validation
}  // namespace lanelet
