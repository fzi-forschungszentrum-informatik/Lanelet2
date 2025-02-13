#pragma once
#include <boost/python.hpp>
#include <sstream>
#include <string>

namespace lanelet {
namespace python {
inline void formatHelper(std::ostream& os) {}

template <typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void formatHelper(std::ostream& os, const std::string& s, const Args&... Args_) {
  if (!s.empty()) {
    os << ", ";
  }
  os << s;
  formatHelper(os, Args_...);
}

template <typename T, typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void formatHelper(std::ostream& os, const T& next, const Args&... Args_) {
  os << ", ";
  os << next;
  formatHelper(os, Args_...);
}

template <typename T, typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void format(std::ostream& os, const T& first, const Args&... Args_) {
  os << first;
  formatHelper(os, Args_...);
}

template <typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
std::string makeRepr(const char* name, const Args&... Args_) {
  std::ostringstream os;
  os << name << '(';
  format(os, Args_...);
  os << ')';
  return os.str();
}

inline std::string repr(const boost::python::object& o) {
  using namespace boost::python;
  object repr = import("builtins").attr("repr");
  return call<std::string>(repr.ptr(), o);
}
}  // namespace python
}  // namespace lanelet