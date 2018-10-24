#pragma once
#include "Parser.h"
#include "Writer.h"

namespace lanelet {
namespace io_handlers {
/**
 * @brief Parser/Writer class for binary files (using boost serialization)
 */
class BinHandler : public Parser, public Writer {
 public:
  // this is necessary in gcc 5.4. In gcc7 a "using" is ok.
  explicit BinHandler(const Projector& projector, const io::Configuration& config = io::Configuration())
      : IOHandler(projector, config) {}

  void write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& /*errors*/) const override;
  std::unique_ptr<LaneletMap> parse(const std::string& filename, ErrorMessages& /*errors*/) const override;

  static constexpr const char* extension() { return ".bin"; }

  static constexpr const char* name() { return "bin_handler"; }
};
}  // namespace io_handlers
}  // namespace lanelet
