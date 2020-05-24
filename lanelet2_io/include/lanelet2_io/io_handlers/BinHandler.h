#pragma once
#include "lanelet2_io/io_handlers/Parser.h"
#include "lanelet2_io/io_handlers/Writer.h"

namespace lanelet {
namespace io_handlers {
/**
 * @brief Writer class for binary files (using boost serialization)
 */
class BinWriter : public Writer {
 public:
  using Writer::Writer;

  void write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& /*errors*/) const override;

  static constexpr const char* extension() { return ".bin"; }

  static constexpr const char* name() { return "bin_handler"; }
};

class BinParser : public Parser {
 public:
  using Parser::Parser;

  std::unique_ptr<LaneletMap> parse(const std::string& filename, ErrorMessages& /*errors*/) const override;

  static constexpr const char* extension() { return ".bin"; }

  static constexpr const char* name() { return "bin_handler"; }
};
}  // namespace io_handlers
}  // namespace lanelet
