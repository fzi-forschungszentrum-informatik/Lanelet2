#pragma once
#include "lanelet2_io/io_handlers/OsmFile.h"
#include "lanelet2_io/io_handlers/Parser.h"
#include "lanelet2_io/io_handlers/Writer.h"

namespace lanelet {
namespace io_handlers {
/**
 * @brief Writer class for osm files
 */
class OsmWriter : public Writer {
 public:
  using Writer::Writer;

  void write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& errors) const override;

  std::unique_ptr<osm::File> toOsmFile(const LaneletMap& laneletMap, ErrorMessages& errors) const;

  static constexpr const char* extension() { return ".osm"; }

  static constexpr const char* name() { return "osm_handler"; }
};
/**

 * @brief Parser class for osm files
 */
class OsmParser : public Parser {
 public:
  using Parser::Parser;

  std::unique_ptr<LaneletMap> parse(const std::string& filename, ErrorMessages& errors) const override;

  std::unique_ptr<LaneletMap> fromOsmFile(const osm::File& file, ErrorMessages& errors) const;

  static constexpr const char* extension() { return ".osm"; }

  static constexpr const char* name() { return "osm_handler"; }
};

}  // namespace io_handlers
}  // namespace lanelet
