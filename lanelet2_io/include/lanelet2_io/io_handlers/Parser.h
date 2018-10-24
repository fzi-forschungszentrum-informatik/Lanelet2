#pragma once
#include <lanelet2_core/LaneletMap.h>
#include <memory>
#include "IoHandler.h"
#include "Projection.h"

namespace lanelet {
namespace io_handlers {
/**
 * @brief Base object for parsers.
 * To create a new parser, you have to do the following steps:
 * 1. Inherit from this parser
 * 2. Overload the parse()-function, the name()-function (from IOHandler) and
 * optionally the extension()-function (from
 * IOHandler)
 * 3. Inherit the constructors (using Parser::Parser)
 * 4. register your parser using the RegisterParser object
 */
class Parser : public virtual IOHandler {
 public:
  Parser() = default;
  using Ptr = std::shared_ptr<Parser>;
  virtual std::unique_ptr<LaneletMap> parse(const std::string& filename, ErrorMessages& errors) const = 0;
};
}  // namespace io_handlers
}  // namespace lanelet
