#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

//! This check looks tags in primitives that should be convertible to bool but aren't.
class BoolTags : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.bool_tags"; }

  Issues operator()(const LaneletMap& map) override;
};

}  // namespace validation
}  // namespace lanelet
