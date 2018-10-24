#pragma once
#include <boost/iterator/iterator_adaptor.hpp>
#include <memory>

namespace lanelet {
namespace internal {
template <typename RetType>
class Converter {
 public:
  template <typename InType>
  RetType& convert(InType& t) const {
    return static_cast<RetType&>(t);
  }
};

template <typename T>
class Converter<std::shared_ptr<const T>> {
 public:
  std::shared_ptr<const T>& convert(std::shared_ptr<T>& t) const {
    val_ = t;
    return val_;
  }

 private:
  //! @todo make this thread-safe
  mutable std::shared_ptr<const T> val_;
};

template <typename T>
class Converter<const std::shared_ptr<const T>> {
 public:
  const std::shared_ptr<const T>& convert(const std::shared_ptr<T>& t) const {
    val_ = t;
    return val_;
  }

 private:
  mutable std::shared_ptr<const T> val_;
};

template <typename RetType>
class PairConverter {
 public:
  template <typename PairType>
  RetType& convert(PairType& p) const {
    return c_.convert(p.second);
  }

 private:
  Converter<RetType> c_;
};

template <typename Iterator, typename RetType, typename Converter = Converter<RetType>>
class TransformIterator
    : public boost::iterator_adaptor<TransformIterator<Iterator, RetType, Converter>, Iterator, RetType> {
  using Base = boost::iterator_adaptor<TransformIterator<Iterator, RetType, Converter>, Iterator, RetType>;
  friend class boost::iterator_core_access;

 public:
  // Init and convert
  TransformIterator() = default;
  TransformIterator(Iterator iter) : Base{iter} {}    // NOLINT
  operator Iterator() const { return this->base(); }  // NOLINT

 private:
  typename Base::reference dereference() const { return converter_.convert(*this->base()); }
  Converter converter_;
};
}  // namespace internal
}  // namespace lanelet
