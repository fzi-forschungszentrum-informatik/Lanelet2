#pragma once
#include <boost/optional.hpp>

namespace lanelet {
// To be replaced by std::optional once c++17 is established
template <typename T>
using Optional = boost::optional<T>;

}  // namespace lanelet
