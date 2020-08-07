#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

//! This check looks for tags that are not part of the lanelet2 specification. This may be very strict, since other tags
//! are not strictly forbidden, but might make sense in some cases.
class UnknownTags : public MapValidator {
 public:
  constexpr static const char* name() { return "mapping.unknown_tags"; }

  Issues operator()(const LaneletMap& map) override;
};

}  // namespace validation
}  // namespace lanelet
