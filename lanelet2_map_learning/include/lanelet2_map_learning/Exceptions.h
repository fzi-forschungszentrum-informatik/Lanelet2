#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>

#include <stdexcept>

namespace lanelet {
/**
 * @brief Thrown when an export to the provided file(name) cannot be done.
 */
class ExportError : public LaneletError {
  using LaneletError::LaneletError;
};

/**
 * @brief Thrown when an there's an error in the graph.
 * It will feature further information.
 */
class MapGraphError : public LaneletError {
  using LaneletError::LaneletError;
};

}  // namespace lanelet
