#pragma once
#include <boost/iterator/iterator_adaptor.hpp>
#include <cassert>

namespace lanelet {
namespace internal {

template <typename IteratorT>
class ReverseAndForwardIterator : public boost::iterator_adaptor<ReverseAndForwardIterator<IteratorT>, IteratorT> {
  using Base = boost::iterator_adaptor<ReverseAndForwardIterator<IteratorT>, IteratorT>;
  friend class boost::iterator_core_access;

 public:
  using Iterator = IteratorT;
  using ReverseIterator = std::reverse_iterator<IteratorT>;

  static ReverseAndForwardIterator reversed(Iterator iter) {
    ReverseAndForwardIterator riter(iter);
    riter.forward_ = false;
    return riter;
  }
  ReverseAndForwardIterator() = default;
  ReverseAndForwardIterator(Iterator iter) : Base(iter) {}  // NOLINT
  ReverseAndForwardIterator(ReverseIterator iter)           // NOLINT
      : Base(iter.base()), forward_{false} {}
  operator Iterator() const { return this->base(); }  // NOLINT

  bool reversed() const { return !forward_; }
  bool forward() const { return forward_; }

 private:
  void increment() { forward() ? ++iter() : --iter(); }
  void decrement() { forward() ? --iter() : ++iter(); }
  void advance(typename Base::difference_type d) { forward() ? iter() += d : iter() -= d; }
  typename Base::reference dereference() const { return forward_ ? *this->base() : *ReverseIterator(this->base()); }

  template <class OtherIterator>
  typename Base::difference_type distance_to(  // NOLINT
      const ReverseAndForwardIterator<OtherIterator>& y) const {
    assert(forward() == y.forward());
    auto d = std::distance(this->base(), y.base());
    return forward() ? d : -d;
  }
  inline IteratorT& iter() { return this->base_reference(); }
  bool forward_{true};
};
}  // namespace internal
}  // namespace lanelet
