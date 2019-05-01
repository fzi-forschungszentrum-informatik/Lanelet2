#pragma once
#include <stdexcept>
#include <vector>

namespace lanelet {

/**
 * @brief Generic lanelet error class.
 * All errors lanelet2 will throw derive from this type
 */
class LaneletError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

/**
 * @brief Thrown when multiple errors occur at the same time
 *
 * Construction is not exception-save, don't throw this on memory errors!
 */
class LaneletMultiError : public LaneletError {
 public:
  using ErrorMessages = std::vector<std::string>;
  explicit LaneletMultiError(const std::string& err) : LaneletError(err), errorMessages{{err}} {}
  explicit LaneletMultiError(ErrorMessages messages = {})
      : LaneletError(combineErrors(messages)), errorMessages(std::move(messages)) {}
  const ErrorMessages errorMessages;  //!< The individual error strings

 private:
  static std::string combineErrors(const ErrorMessages& m) {
    std::string result;
    for (const auto& message : m) {
      result += message;
      result += '\n';
    }
    return result;
  }
};

/**
 * @brief Thrown when an attribute has been queried that does not exist.
 */
class NoSuchAttributeError : public LaneletError {
  using LaneletError::LaneletError;
};

/**
 * @brief Thrown when an element is not part of the map
 */
class NoSuchPrimitiveError : public LaneletError {
  using LaneletError::LaneletError;
};

/**
 * @brief Thrown when a function was called with invalid input arguments.
 */
class InvalidInputError : public LaneletError {
  using LaneletError::LaneletError;
};

/**
 * @brief Thrown when a geometric operation is not valid.
 */
class GeometryError : public LaneletError {
  using LaneletError::LaneletError;
};

/**
 * @brief Thrown when the state of a lanelet object is invalid
 * E.g. when an linestring has no points or member pointers are NULL
 */
class InvalidObjectStateError : public LaneletError {
  using LaneletError::LaneletError;
};

/**
 * @brief SharedPtrs in lanelet2 must never point to null.
 * If this is violated, this exception is thrown (usually checked at object
 * construction).
 */
class NullptrError : public LaneletError {
  using LaneletError::LaneletError;
};

}  // namespace lanelet
