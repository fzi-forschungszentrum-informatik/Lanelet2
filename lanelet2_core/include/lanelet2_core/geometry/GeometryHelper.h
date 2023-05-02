#pragma once
#include <memory>

namespace lanelet {
namespace helper {
/**
 * @brief apply a function for each pair of adjacent elements in a range
 * @param first first element in range
 * @param end past-the-end iterator
 * @param f function/lambda to apply (defines operator())
 *
 * This is useful for iterating over a the segments in a LineString.
 */
template <typename FwIter, typename Func>
void forEachPair(FwIter first, FwIter end, Func f) {
  if (first != end) {
    FwIter second = first;
    ++second;
    for (; second != end; ++first, ++second) {
      f(*first, *second);
    }
  }
}

/**
 * @brief Apply a function of all pairs in a sequenc in sorted order until a
 * predicate returns true
 * @param first begin of the range to iterate over
 * @param end end of the range to iterate over
 * @param f function returning something convertible to bool. If the result is
 * true, the iteration is stopped
 * @return Iterator to the first element of the pair that returned true or end
 *
 * Works similar to forEachPair but stops if f returns true. Useful for
 * implementing a search on segments.
 */
template <typename FwIter, typename Func>
FwIter forEachPairUntil(FwIter first, FwIter end, Func f) {
  if (first != end) {
    FwIter second = first;
    ++second;
    for (; second != end; ++first, ++second) {
      auto res = f(*first, *second);
      if (res) {
        return first;
      }
    }
  }
  return end;
}
}  // namespace helper
}  // namespace lanelet