#pragma once
#include <lanelet2_core/LaneletMap.h>
#include <memory>
#include "IoHandler.h"
#include "Projection.h"

namespace lanelet {
namespace io_handlers {
/**
 * @brief Base object for writers.
 * To create a new writer, you have to do the following steps:
 * 1. Inherit from this writer
 * 2. Overload the parse()-function, the name()-function (from IOHandler) and
 * optionally the extension()-function (from
 * IOHandler)
 * 3. Inherit the constructors (using Writer::Writer)
 * 4. register your writer using the Registerwriter object
 */
class Writer : public virtual IOHandler {
 public:
  using Ptr = std::shared_ptr<Writer>;

  virtual void write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& errors) const = 0;
};
}  // namespace io_handlers
}  // namespace lanelet
