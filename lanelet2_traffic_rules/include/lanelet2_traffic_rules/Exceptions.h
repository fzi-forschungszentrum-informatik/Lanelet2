#pragma once
#include <lanelet2_core/Exceptions.h>

namespace lanelet {
//! Thrown when unable to interpret details of a traffic rules (i.e. unknown
//! tags, etc)
class InterpretationError : public LaneletError {
  using LaneletError::LaneletError;
};
}  // namespace lanelet
