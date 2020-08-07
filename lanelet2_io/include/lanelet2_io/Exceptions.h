#pragma once
#include <lanelet2_core/Exceptions.h>

#include <stdexcept>

namespace lanelet {
/**
 * @brief Generic error for all errors in this module
 *
 * Threse are derived from multi error, because often multiple issues occur at
 * the same time when reading or writing a map.
 */
class IOError : public LaneletMultiError {
  using LaneletMultiError::LaneletMultiError;
};

/**
 * @brief Error for not existent filepaths
 */
class FileNotFoundError : public IOError {
  using IOError::IOError;
};

/**
 * @brief Error for an unsupported extension
 */
class UnsupportedExtensionError : public IOError {
  using IOError::IOError;
};

/**
 * @brief Error thrown if an unsupported handler (parser/writer) has been
 * specified
 */
class UnsupportedIOHandlerError : public IOError {
  using IOError::IOError;
};

/**
 * @brief Error thrown if some error occured during the parsing of the file
 */
class ParseError : public IOError {
  using IOError::IOError;
};

/**
 * @brief Error thown if some error occurd during writing of a map
 */
class WriteError : public IOError {
  using IOError::IOError;
};

/**
 * @brief Thrown by the projector classes if projection from lat/lon to x/y
 * fails
 */
class ForwardProjectionError : public ParseError {
  using ParseError::ParseError;
};

/**
 * @brief Thrown by the projector classes if projection from x/y to lat/lon
 * fails
 */
class ReverseProjectionError : public WriteError {
  using WriteError::WriteError;
};

/**
 * @brief Thrown when a user attempts to load a map with georeferenced data without providing an origin.
 */
class DefaultProjectionNotAllowedError : public IOError {
  using IOError::IOError;
};
}  // namespace lanelet
