#pragma once
#include <lanelet2_core/Forward.h>

#include <sstream>
#include <vector>

namespace lanelet {
namespace validation {
enum class Severity { Error, Warning, Info };
enum class Primitive { Point, LineString, Polygon, Lanelet, Area, RegulatoryElement, Primitive };

inline const char* toString(Severity severity) {
  switch (severity) {
    case Severity::Error:
      return "Error";
    case Severity::Warning:
      return "Warning";
    case Severity::Info:
      return "info";
  }
  return "";
}

inline const char* toString(Primitive primitive) {
  switch (primitive) {
    case Primitive::Point:
      return "point";
    case Primitive::LineString:
      return "linestring";
    case Primitive::Polygon:
      return "polygon";
    case Primitive::Lanelet:
      return "lanelet";
    case Primitive::Area:
      return "area";
    case Primitive::RegulatoryElement:
      return "regulatory element";
    case Primitive::Primitive:
      return "primitive";
  }
  return "";
}

struct Issue {
  Issue() = default;
  Issue(Severity severity, std::string message) : severity{severity}, message{std::move(message)} {}
  Issue(Severity severity, Primitive primitive, Id id, std::string message)
      : severity{severity}, primitive{primitive}, id{id}, message{std::move(message)} {}
  std::string buildReport() const {
    std::stringstream ss;
    ss << toString(severity) << ": ";
    if (id != InvalId) {
      ss << toString(primitive) << " " << std::to_string(id) << " ";
    }
    ss << message;
    return ss.str();
  }

  Severity severity{Severity::Error};     //!< Severity class of issue
  Primitive primitive{Primitive::Point};  //!< Type of primitive that caused the issue
  Id id{InvalId};                         //!< the id of primitive that caused the issue
  std::string message;                    //!< Message to be displayed
};

using Issues = std::vector<Issue>;

}  // namespace validation
}  // namespace lanelet
