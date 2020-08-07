#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

//! This check looks tags that should be set on a primitive but aren't
class MandatoryTags : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.mandatory_tags"; }

  Issues operator()(const LaneletMap& map) override;
};

}  // namespace validation
}  // namespace lanelet
