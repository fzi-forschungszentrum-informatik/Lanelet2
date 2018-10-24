#pragma once
#include "OsmFile.h"
#include "Parser.h"
#include "Writer.h"

namespace lanelet {
namespace io_handlers {
/**
 * @brief Parser/Writer class for osm files
 */
class OsmHandler : public Parser, public Writer {
 public:
  explicit OsmHandler(const Projector& projector, const io::Configuration& config = io::Configuration())
      : IOHandler(projector, config) {}

  void write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& errors) const override;
  std::unique_ptr<LaneletMap> parse(const std::string& filename, ErrorMessages& errors) const override;

  std::unique_ptr<osm::File> toOsmFile(const LaneletMap& laneletMap, ErrorMessages& errors) const;
  std::unique_ptr<LaneletMap> fromOsmFile(const osm::File& file, ErrorMessages& errors) const;

  static constexpr const char* extension() { return ".osm"; }

  static constexpr const char* name() { return "osm_handler"; }
};
}  // namespace io_handlers
}  // namespace lanelet
