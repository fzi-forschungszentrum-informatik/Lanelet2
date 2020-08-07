#pragma once
#include <lanelet2_core/LaneletMap.h>

#include <iostream>
#include <memory>

#include "lanelet2_io/Projection.h"
#include "lanelet2_io/io_handlers/IoHandler.h"

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
class Writer : public IOHandler {
 public:
  Writer() = default;
  using IOHandler::IOHandler;
  using Ptr = std::shared_ptr<Writer>;

  virtual void write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& errors) const = 0;

  // IOHandler interface
 private:
  void handleDefaultProjector() const final {
    std::cout << "Default origin should not be used when writing into a format that uses georeferenced lat/lon "
                 "coordinates. Will continue to write the map, but the data will be dislocated and deformed"
              << std::endl;
  }
};
}  // namespace io_handlers
}  // namespace lanelet
