#pragma once
#include <algorithm>
#include <boost/iterator/iterator_adaptor.hpp>
#include <iostream>

namespace lanelet {
namespace internal {

/**
 * @brief This iterator iterates over a container of containers as if it was one
 * single container. Succeeding elements that are equal are ignored.
 *
 * This container is used by the compound types of lanelet2, e.g.
 * CompoundLineString2d.
 *
 * The iterator is only bidirectional, i.e. it allows no random access in O(0).
 */
template <typename ContainerT>
class UniqueCompoundIterator
    : public boost::iterator_facade<UniqueCompoundIterator<ContainerT>,
                                    std::remove_reference_t<decltype(*ContainerT().begin()->begin())>,
                                    std::random_access_iterator_tag> {
  using Base = boost::iterator_facade<UniqueCompoundIterator<ContainerT>,
                                      std::remove_reference_t<decltype(*ContainerT().begin()->begin())>,
                                      std::random_access_iterator_tag>;

 public:
  using ItOuter = std::decay_t<decltype(ContainerT().begin())>;
  using ItInner = std::decay_t<decltype(ItOuter()->begin())>;
  UniqueCompoundIterator() = default;
  static UniqueCompoundIterator begin(ContainerT& c) {
    auto itBegin = firstNonempty(c);
    if (itBegin != c.end()) {
      return UniqueCompoundIterator(c, itBegin, itBegin->begin());
    }
    return UniqueCompoundIterator(c, itBegin, ItInner());
  }
  static UniqueCompoundIterator end(ContainerT& c) { return UniqueCompoundIterator(c, c.end(), {}); }

 private:
  friend class boost::iterator_core_access;
  UniqueCompoundIterator(ContainerT& c, ItOuter itOuter, ItInner itInner)
      : c_{&c}, itOuter_{itOuter}, itInner_{itInner} {}
  static ItOuter firstNonempty(ContainerT& c) {
    return std::find_if(c.begin(), c.end(), [](auto& c) { return !c.empty(); });
  }

  void increment() {
    auto& old = *(*this);
    auto endIt = end(*c_);
    do {
      incrementOne();
    } while ((*this) != endIt && old == *(*this));
  }

  void decrement() {
    decrementOne();
    const auto startIt = begin(*c_);
    if (*this == startIt) {
      return;
    }
    auto next = *this;
    do {
      *this = next;
      next.decrementOne();
      if (*next != *(*this)) {
        return;
      }
    } while (next != startIt);
    *this = next;
  }

  void incrementOne() {
    if (itOuter_->end() == std::next(itInner_)) {
      do {
        ++itOuter_;
      } while (itOuter_ != c_->end() && itOuter_->empty());
      if (itOuter_ == c_->end()) {
        itInner_ = ItInner();
      } else {
        itInner_ = itOuter_->begin();
      }
    } else {
      ++itInner_;
    }
  }

  void decrementOne() {
    if (itOuter_ == c_->end() || itInner_ == itOuter_->begin()) {
      const auto begin = firstNonempty(*c_);
      do {
        --itOuter_;
      } while (itOuter_ != begin && itOuter_->empty());
      itInner_ = std::prev(itOuter_->end());
    } else {
      --itInner_;
    }
  }

  void advance(typename Base::difference_type d) {
    if (d > 0) {
      for (; d != 0; d--) {
        increment();
      }
    } else {
      for (; d != 0; d++) {
        decrement();
      }
    }
  }

  typename Base::difference_type distance_to(  // NOLINT
      UniqueCompoundIterator other) const {
    typename Base::difference_type d{};
    auto cp = *this;
    if (other.itOuter_ > itOuter_ || (other.itOuter_ == itOuter_ && other.itInner_ > itInner_)) {
      while (!cp.equal(other)) {
        cp.increment();
        d++;
      }
    } else {
      while (!cp.equal(other)) {
        other.increment();
        d--;
      }
    }
    return d;
  }

  bool equal(UniqueCompoundIterator other) const { return itOuter_ == other.itOuter_ && itInner_ == other.itInner_; }

  decltype(auto) dereference() const { return *itInner_; }

  ContainerT* c_{nullptr};
  ItOuter itOuter_;
  ItInner itInner_;
};

}  // namespace internal
}  // namespace lanelet
