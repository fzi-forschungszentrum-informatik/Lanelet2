#pragma once
#include <lanelet2_core/Attribute.h>

#include <map>
#include <string>
namespace lanelet {
namespace io {
using Configuration = std::map<std::string, Attribute>;
}  // namespace io
}  // namespace lanelet
