#pragma once

#include <algorithm>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/type_traits/remove_cv.hpp>
#include <boost/variant/get.hpp>
#include <memory>
#include <numeric>
#include <type_traits>
#include <vector>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/utility/Optional.h"

namespace lanelet {

namespace utils {
/**
 * @brief Wrapper around std::accumulate for summing up the result of Func over
 * a range.
 */
template <typename Range, typename Func>
auto sum(Range&& r, Func f);

namespace detail {
template <bool Move>
struct MoveIf {};

template <>
struct MoveIf<true> {
  template <typename Iter>
  auto operator()(Iter it) {
    return std::make_move_iterator(it);
  }
};

template <>
struct MoveIf<false> {
  template <typename Iter>
  auto operator()(Iter it) {
    return it;
  }
};

template <typename ContainerT>
struct ReserveIfNecessary {
  void operator()(ContainerT& /*c*/, size_t /*size*/) const {}
};

template <typename T>
struct ReserveIfNecessary<std::vector<T>> {
  void operator()(std::vector<T>& c, size_t size) const { c.reserve(size); }
};

template <typename VectorT>
inline VectorT createReserved(size_t size) {
  VectorT vector;
  ReserveIfNecessary<VectorT>()(vector, size);
  return vector;
}

template <typename VectorT, typename ContainerT>
VectorT concatenate(ContainerT&& c) {
  VectorT conced;
  const auto size = sum(c, [](auto& elem) { return elem.size(); });
  ReserveIfNecessary<VectorT>()(conced, size);
  MoveIf<std::is_rvalue_reference<decltype(c)>::value> move;
  for (auto& elem : c) {
    conced.insert(conced.end(), move(elem.begin()), move(elem.end()));
  }
  return conced;
}

template <typename VectorT, typename ContainerT, typename Func>
VectorT concatenate(ContainerT&& c, Func f) {
  VectorT conced;
  MoveIf<true> move;
  for (auto& elem : c) {
    auto value = f(elem);
    conced.insert(conced.end(), move(value.begin()), move(value.end()));
  }
  return conced;
}

template <typename ContainerT, typename Func>
auto concatenateRange(ContainerT&& c, Func f) {
  auto size = sum(c, [f](auto&& elem) {
    auto range = f(elem);
    return std::distance(range.first, range.second);
  });
  using RetT = typename decltype(f(*c.begin()).first)::value_type;
  std::vector<RetT> ret;
  ret.reserve(size);
  MoveIf<std::is_rvalue_reference<decltype(c)>::value> move;
  for (auto&& elem : c) {
    auto range = f(elem);
    ret.insert(ret.end(), move(range.first), move(range.second));
  }
  return ret;
}

template <typename ContainerT, typename Func, typename AllocatorT>
auto concatenateRange(ContainerT&& c, Func f, const AllocatorT& alloc) {
  auto size = sum(c, [f](auto&& elem) {
    auto range = f(elem);
    return std::distance(range.first, range.second);
  });
  using RetT = typename decltype(f(*c.begin()).first)::value_type;
  std::vector<RetT, AllocatorT> ret(alloc);
  ret.reserve(size);
  MoveIf<std::is_rvalue_reference<decltype(c)>::value> move;
  for (auto&& elem : c) {
    auto range = f(elem);
    ret.insert(ret.end(), move(range.first), move(range.second));
  }
  return ret;
}

template <typename Container, typename Func>
auto transform(Container&& c, Func f) {
  using RetT = std::decay_t<decltype(f(*c.begin()))>;
  std::vector<RetT> transformed;
  transformed.reserve(c.size());
  std::transform(c.begin(), c.end(), std::back_inserter(transformed), f);
  return transformed;
}
}  // namespace detail

using detail::createReserved;

//! Compares ids of primitives. Can be used for some stl algorithms
template <typename PrimitiveT>
constexpr bool idLess(const PrimitiveT& p1, const PrimitiveT& p2) {
  return p1.id() < p2.id();
}

//! Compares ids of primitives. Can be used for some sorted stl containers
template <typename T>
struct IdLess {
  constexpr bool operator()(const T& a, const T& b) { return idLess(a, b); }
};

template <typename T1, typename T2>
inline auto orderedPair(T1& first, T2& second) {
  return (first.id() < second.id()) ? std::make_pair(first, second) : std::make_pair(second, first);
}

template <class T>
const T& addConst(T& t) {
  return t;
}

template <class T>
std::shared_ptr<const T> addConst(std::shared_ptr<T> t) {
  return std::const_pointer_cast<const T>(t);
}

template <class T>
std::shared_ptr<T> removeConst(std::shared_ptr<const T> t) {
  return std::const_pointer_cast<T>(t);
}

template <class T>
T& removeConst(const T& t) {
  return const_cast<T&>(t);  // NOLINT
}

template <typename Container>
Container invert(const Container& cont) {
  Container cOut;
  std::reverse_copy(cont.begin(), cont.end(), std::back_inserter(cOut));
  return cOut;
}

template <typename Iterator, typename Func>
auto transform(Iterator begin, Iterator end, const Func f) {
  using ValueT = decltype(*begin);
  using RetT = std::decay_t<decltype(f(*begin))>;
  std::vector<RetT> transformed;
  transformed.reserve(std::distance(begin, end));
  std::transform(begin, end, std::back_inserter(transformed), [&f](ValueT elem) { return f(elem); });
  return transformed;
}

template <typename ContainerT, typename ValueT>
auto find(ContainerT&& c, const ValueT& val) {
  using RetT = Optional<std::decay_t<decltype(*std::begin(c))>>;
  auto it = std::find(std::begin(c), std::end(c), val);
  if (it != std::end(c)) {
    return RetT(*it);
  }
  return RetT();
}

template <typename ContainerT, typename Func>
auto findIf(ContainerT&& c, Func f) {
  using RetT = Optional<std::decay_t<decltype(*std::begin(c))>>;
  auto it = std::find_if(std::begin(c), std::end(c), f);
  if (it != std::end(c)) {
    return RetT(*it);
  }
  return RetT();
}

template <typename Container, typename Func>
auto transform(Container&& c, Func f) {
  return detail::transform(std::forward<Container>(c), f);
}

template <typename T, typename Func>
auto transform(std::initializer_list<T>&& c, Func f) {
  return detail::transform(std::move(c), f);
}

template <typename Container, typename Predicate>
bool anyOf(const Container& c, Predicate&& p) {
  return std::any_of(c.begin(), c.end(), p);
}

template <typename Container, typename Value>
bool contains(const Container& c, const Value& v) {
  return std::find(c.begin(), c.end(), v) != c.end();
}

template <typename Container, typename Func>
void forEach(Container&& c, Func&& f) {
  std::for_each(c.begin(), c.end(), std::forward<Func>(f));
}

template <typename OutT, typename InT>
auto transformSharedPtr(const std::vector<std::shared_ptr<InT>>& v) {
  std::vector<std::shared_ptr<OutT>> transformed;
  transformed.reserve(v.size());
  for (const auto& elem : v) {
    auto cast = std::dynamic_pointer_cast<OutT>(elem);
    if (cast) {
      transformed.push_back(std::move(cast));
    }
  }
  return transformed;
}

/**
 * @brief Wrapper around std::accumulate for summing up the result of Func over
 * a range.
 */
template <typename Range, typename Func>
auto sum(Range&& r, Func f) {
  using ValT = decltype(f(*std::begin(r)));
  return std::accumulate(std::begin(r), std::end(r), ValT(), [&f](auto v, auto&& elem) { return v + f(elem); });
}

/**
 * @brief overload assuming that c is a container of containers.
 * The return type will be the type of the inner container.
 *
 * For containers holding heavy objects consider moving the container into this.
 */
template <typename ContainerT>
auto concatenate(ContainerT&& c) {
  using VectorT = std::decay_t<decltype(*std::begin(c))>;
  return detail::concatenate<VectorT>(std::forward<ContainerT>(c));
}

template <typename T>
auto concatenate(std::initializer_list<T>&& c) {
  using VectorT = std::decay_t<decltype(*std::begin(c))>;
  return detail::concatenate<VectorT>(std::move(c));
}

/**
 * @brief create one big vector from a vector and a function that returns a
 * vector for each element in this vector
 */
template <typename ContainerT, typename Func>
auto concatenate(ContainerT&& c, Func f) {
  using VectorT = std::decay_t<decltype(f(*c.begin()))>;
  return detail::concatenate<VectorT>(std::forward<ContainerT>(c), f);
}

template <typename T, typename Func>
auto concatenate(std::initializer_list<T>&& c, Func f) {
  using VectorT = std::decay_t<decltype(f(*c.begin()))>;
  return detail::concatenate<VectorT>(std::move(c), f);
}

/**
 * @brief Similar to concatenate but expects Func to return a pair of begin and
 * end of the range to concatenated.
 *
 * Significantly more efficient than the above but requires that the range is
 * already present in memory. A use case is a vector of vectors.
 */
template <typename ContainerT, typename Func>
auto concatenateRange(ContainerT&& c, Func f) {
  return detail::concatenateRange(std::forward<ContainerT>(c), f);
}

template <typename T, typename Func>
auto concatenateRange(std::initializer_list<T>&& c, Func f) {
  return detail::concatenateRange(std::move(c), f);
}

/// Overload for an allocator (e.g. for Eigen::aligned_allocator)
template <typename ContainerT, typename Func, typename AllocatorT>
auto concatenateRange(ContainerT&& c, Func f, const AllocatorT& alloc) {
  return detail::concatenateRange(std::forward<ContainerT>(c), f, alloc);
}

template <typename T, typename Func, typename AllocatorT>
auto concatenateRange(std::initializer_list<T>&& c, Func f, const AllocatorT& alloc) {
  return detail::concatenateRange(std::move(c), f, alloc);
}

template <typename T, typename Variant>
std::vector<T> getVariant(const std::vector<Variant>& v) {
  std::vector<T> s;
  s.reserve(v.size());
  for (const auto& e : v) {
    const auto* t = boost::get<typename T::MutableType>(&e);
    if (t) {
      s.push_back(*t);
    }
  }
  return s;
}

/**
 * @brief compares two c-strings at compile time. Do not use this for run time.
 * @return true if strings match.
 */
constexpr bool strequal(char const* lhs, char const* rhs) {
  while (*lhs != 0 && *rhs != 0) {
    if (*lhs++ != *rhs++) {  // NOLINT
      return false;
    }
  }
  return *lhs == 0 && *rhs == 0;
}

/**
 * @brief transforms a vector of weak_ptrs to a vector of shared_ptrs
 *
 * Expired elements are discarded.
 */
template <typename WeakT>
auto strong(std::vector<WeakT> v) {
  std::vector<decltype(v.front().lock())> s;
  s.reserve(v.size());
  for (const auto& e : v) {
    if (!e.expired()) {
      s.push_back(e.lock());
    }
  }
  return s;
}

//! Tests if a double is close to zero within a small margin
inline bool nearZero(double d) { return std::abs(d) < 1e-8; }

}  // namespace utils

namespace internal {
// See
// https://stackoverflow.com/questions/765148/how-to-remove-constness-of-const-iterator
template <typename Container, typename ConstIterator>
typename Container::iterator removeConst(Container& c, ConstIterator it) {
  return c.erase(it, it);
}
}  // namespace internal
}  // namespace lanelet
