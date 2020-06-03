#pragma once
#include <algorithm>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace lanelet {

namespace detail {
template <typename ArrayT, ArrayT Arr>
class ArrayView {
 public:
  static constexpr auto begin() { return std::begin(Arr); }
  static constexpr auto end() { return std::end(Arr); }

  static constexpr auto findValue(const decltype(Arr[0].second)& val) {
    auto it = begin();
    for (; it != end(); ++it) {
      if (it->second == val) {
        break;
      }
    }
    return it;
  }
  static constexpr auto findKey(const decltype(Arr[0].first)& val) {
    auto it = begin();
    for (; it != end(); ++it) {
      if (strcmp(it->first, val) == 0) {
        break;
      }
    }
    return it;
  }
};

template <typename Iterator, typename Map>
std::vector<Iterator> copyIterators(const std::vector<Iterator>& oldV, const Map& oldM, Map& newM) {
  std::vector<Iterator> newV(oldV.size(), newM.end());
  for (auto i = 0u; i < oldV.size(); ++i) {
    if (oldV[i] != oldM.end()) {
      newV[i] = newM.find(oldV[i]->first);
    }
  }
  return newV;
}

template <typename Iterator, typename Vector>
void replaceIterator(Vector& v, const Iterator& replace, const Iterator& by) {
  for (auto& elem : v) {
    if (elem == replace) {
      elem = by;
    }
  }
}
}  // namespace detail

/**
 * @brief A hybrid map is just like a normal map with keys as string, but
 * elements can also be accessed using an enum for the keys. This is much faster
 * than using strings for the lookup.
 * @tparam Enum an enum with *continuous* values. The last element must be named
 * "End".
 */
template <typename ValueT, typename PairArrayT, PairArrayT PairArray>
class HybridMap {
  using Array = detail::ArrayView<PairArrayT, PairArray>;

 public:
  using Map = std::map<std::string, ValueT>;
  using Vec = std::vector<typename Map::iterator>;
  using Enum = std::decay_t<decltype(PairArray[0].second)>;

  using key_type = typename Map::key_type;                // NOLINT
  using mapped_type = typename Map::mapped_type;          // NOLINT
  using iterator = typename Map::iterator;                // NOLINT
  using const_iterator = typename Map::const_iterator;    // NOLINT
  using value_type = typename Map::value_type;            // NOLINT
  using difference_type = typename Map::difference_type;  // NOLINT
  using size_type = typename Map::size_type;              // NOLINT
  HybridMap() noexcept = default;
  HybridMap(HybridMap&& rhs) noexcept : m_(std::move(rhs.m_)), v_{std::move(rhs.v_)} {
    // move invalidates no iterators except end
    detail::replaceIterator(v_, rhs.m_.end(), m_.end());
  }
  HybridMap& operator=(HybridMap&& rhs) noexcept {
    m_ = std::move(rhs.m_);
    v_ = std::move(rhs.v_);
    // move invalidates no iterators except end
    detail::replaceIterator(v_, rhs.m_.end(), m_.end());
    return *this;
  }
  HybridMap(const HybridMap& rhs) : m_{rhs.m_}, v_{detail::copyIterators(rhs.v_, rhs.m_, m_)} {}
  HybridMap& operator=(const HybridMap& rhs) {
    m_ = rhs.m_;
    v_ = detail::copyIterators(rhs.v_, rhs.m_, m_);
    return *this;
  }
  HybridMap(const std::initializer_list<std::pair<const std::string, ValueT>>& list) {
    for (const auto& item : list) {
      insert(item);
    }
  }
  template <typename InputIterator>
  HybridMap(InputIterator begin, InputIterator end) {
    for (; begin != end; ++begin) {
      insert(*begin);
    }
  }

  ~HybridMap() noexcept = default;

  iterator find(const key_type& k) { return m_.find(k); }
  iterator find(Enum k) {
    const auto pos = static_cast<size_t>(k);
    return v_.size() < pos + 1 ? m_.end() : v_[pos];
  }
  const_iterator find(const key_type& k) const { return m_.find(k); }
  const_iterator find(Enum k) const {
    const auto pos = static_cast<size_t>(k);
    return v_.size() < pos + 1 ? m_.end() : v_[pos];
  }

  iterator begin() { return m_.begin(); }
  iterator end() { return m_.end(); }
  const_iterator begin() const { return m_.begin(); }
  const_iterator end() const { return m_.end(); }

  std::pair<iterator, bool> insert(const value_type& v) {
    auto it = m_.insert(v);
    if (it.second) {
      updateV(it.first);
    }
    return it;
  }

  iterator insert(const_iterator hint, const value_type& v) {
    auto it = m_.insert(hint, v);
    updateV(it);
    return it;
  }

  iterator erase(const_iterator pos) {
    auto itV = std::find(v_.begin(), v_.end(), pos);
    if (itV != v_.end()) {
      *itV = m_.end();
    }
    return m_.erase(pos);
  }

  size_type erase(const std::string& k) {
    auto it = find(k);
    if (it == end()) {
      return 0;
    }
    erase(it);
    return 1;
  }

  ValueT& operator[](const key_type& k) {
    auto it = find(k);
    if (it == m_.end()) {
      it = insert({k, mapped_type()}).first;
    }
    return it->second;
  }
  ValueT& operator[](const Enum& k) {
    auto it = find(k);
    auto end = m_.end();
    if (it == end) {
      auto attr = Array::findValue(k);
      it = insert({attr->first, mapped_type()}).first;
    }
    return it->second;
  }

  const ValueT& at(const key_type& k) const {
    auto it = find(k);
    if (it == m_.end()) {
      throw std::out_of_range(std::string("Could not find ") + k);
    }
    return it->second;
  }
  const ValueT& at(const Enum& k) const {
    auto it = find(k);
    if (it == m_.end()) {
      throw std::out_of_range(std::string("Could not find ") + std::to_string(static_cast<int>(k)));
    }
    return it->second;
  }

  ValueT& at(const key_type& k) {
    auto it = find(k);
    if (it == m_.end()) {
      throw std::out_of_range(std::string("Could not find ") + k);
    }
    return it->second;
  }
  ValueT& at(const Enum& k) {
    auto it = find(k);
    if (it == m_.end()) {
      throw std::out_of_range(std::string("Could not find ") + std::to_string(static_cast<int>(k)));
    }
    return it->second;
  }

  void clear() { m_.clear(), v_.clear(); }

  bool empty() const { return m_.empty(); }
  size_t size() const { return m_.size(); }

  auto key_comp() const { return m_.key_comp(); }    // NOLINT
  auto value_comp() const { return m_.key_comp(); }  // NOLINT

  bool operator==(const HybridMap& other) const { return this->m_ == other.m_; }
  bool operator!=(const HybridMap& other) const { return !(*this == other); }

 private:
  void updateV(iterator it) {
    auto attr = Array::findKey(it->first.c_str());
    if (attr != std::end(PairArray)) {  // NOLINT
      const auto pos = static_cast<size_t>(attr->second);
      if (v_.size() < pos + 1) {
        v_.resize(pos + 1, m_.end());
      }
      v_[pos] = it;
    }
  }
  Map m_;
  Vec v_;
};

template <typename Value, typename Enum, const std::pair<const char*, const Enum> Lookup[]>
std::ostream& operator<<(std::ostream& stream, HybridMap<Value, Enum, Lookup> map) {
  stream << "{";
  for (const auto& elem : map) {
    stream << "{" << elem.first << ": " << elem.second << "}";
  }
  return stream << "}";
}
}  // namespace lanelet
