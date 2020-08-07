#pragma once
#include <utility>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/BoundingBox.h"
#include "lanelet2_core/primitives/Point.h"
#include "lanelet2_core/primitives/Primitive.h"
#include "lanelet2_core/utility/ReverseAndForwardIterator.h"
#include "lanelet2_core/utility/TransformIterator.h"
#include "lanelet2_core/utility/Utilities.h"

namespace lanelet {

using BasicLineString2d = BasicPoints2d;
using BasicLineString3d = BasicPoints3d;

template <typename PointT>
using Segment = std::pair<PointT, PointT>;

using Segment2d = Segment<Point2d>;
using ConstSegment2d = Segment<ConstPoint2d>;
using Segment3d = Segment<Point3d>;
using ConstSegment3d = Segment<ConstPoint3d>;
using BasicSegment2d = Segment<BasicPoint2d>;
using BasicSegment3d = Segment<BasicPoint3d>;

namespace traits {
template <>
struct PrimitiveTraits<BasicLineString2d> {
  using ConstType = BasicLineString2d;
  using MutableType = BasicLineString2d;
  using TwoDType = BasicLineString2d;
  using ThreeDType = BasicLineString3d;
  using Category = LineStringTag;
};
template <>
struct LineStringTraits<BasicLineString2d> : public PrimitiveTraits<BasicLineString2d> {
  using PointType = BasicPoint2d;
  using HybridType = BasicLineString2d;
};
template <>
struct PrimitiveTraits<BasicLineString3d> {
  using ConstType = BasicLineString3d;
  using MutableType = BasicLineString3d;
  using TwoDType = BasicLineString2d;
  using ThreeDType = BasicLineString3d;
  using Category = LineStringTag;
};
template <>
struct LineStringTraits<BasicLineString3d> : public PrimitiveTraits<BasicLineString3d> {
  using PointType = BasicPoint3d;
  using HybridType = BasicLineString3d;
};
template <typename PointT>
struct PrimitiveTraits<Segment<PointT>> {
  using ConstType = Segment<ConstPrimitiveType<PointT>>;
  using MutableType = Segment<MutablePrimitiveType<PointT>>;
  using TwoDType = Segment<TwoD<PointT>>;
  using ThreeDType = Segment<ThreeD<PointT>>;
  using Category = LineStringTag;
};
template <typename PointT>
struct LineStringTraits<Segment<PointT>> : public PrimitiveTraits<Segment<PointT>> {
  using PointType = PointT;
  using HybridType = Segment<traits::BasicPointT<PointT>>;
};
template <>
inline BasicLineString2d to2D<BasicLineString3d>(const BasicLineString3d& primitive) {
  BasicLineString2d ls2d(primitive.size());
  std::transform(primitive.begin(), primitive.end(), ls2d.begin(), utils::to2D<BasicPoint3d>);
  return ls2d;
}
}  // namespace traits

namespace internal {
template <typename PointT>
struct SelectLsIterator {
  using Iterator = internal::TransformIterator<internal::ReverseAndForwardIterator<Points3d::iterator>, PointT>;
};
template <typename PointT>
struct SelectLsIterator<const PointT> {
  using Iterator =
      internal::TransformIterator<internal::ReverseAndForwardIterator<Points3d::const_iterator>, const PointT>;
};
template <>
struct SelectLsIterator<Point3d> {
  using Iterator = internal::ReverseAndForwardIterator<Points3d::iterator>;
};
template <>
struct SelectLsIterator<const Point3d> {
  using Iterator = internal::ReverseAndForwardIterator<Points3d::const_iterator>;
};

template <typename T>
using SelectLsIteratorT = typename SelectLsIterator<T>::Iterator;

template <typename PointT>
struct SelectBasicLineString {};

template <>
struct SelectBasicLineString<BasicPoint2d> {
  using Type = BasicLineString2d;
};

template <>
struct SelectBasicLineString<BasicPoint3d> {
  using Type = BasicLineString3d;
};

template <typename T>
using SelectBasicLineStringT = typename SelectBasicLineString<T>::Type;

inline Points3d::const_iterator pointIter(internal::ReverseAndForwardIterator<Points3d::iterator> it) {
  return Points3d::iterator(it);
}
template <typename T>
struct SelectInsertIterator {
  using Type = internal::TransformIterator<T, Point3d>;
};
template <>
struct SelectInsertIterator<typename SelectLsIterator<Point2d>::Iterator> {
  using Type = internal::ReverseAndForwardIterator<Points3d::iterator>;
};

}  // namespace internal

//! Common data management class for all LineString primitives.
//! @ingroup DataObjects
class LineStringData : public PrimitiveData {
 public:
  explicit LineStringData(Id id) : PrimitiveData(id) {}
  LineStringData(Id id, Points3d points, AttributeMap attributes)
      : PrimitiveData(id, std::move(attributes)), points_(std::move(points)) {}
  // NOLINTNEXTLINE
  using iterator = internal::ReverseAndForwardIterator<Points3d::iterator>;
  // NOLINTNEXTLINE
  using const_iterator = internal::ReverseAndForwardIterator<Points3d::const_iterator>;

  /**
   * @brief returns a reference to the points
   *
   * This ignores the "inverted" property
   */
  Points3d& points() { return points_; }

  template <typename IteratorT = const_iterator>
  IteratorT begin(bool inverted) const noexcept {
    return inverted ? IteratorT(const_iterator(points_.rbegin())) : IteratorT(const_iterator(points_.begin()));
  }

  template <typename IteratorT = iterator>
  IteratorT begin(bool inverted) noexcept {
    return inverted ? IteratorT(iterator(points_.rbegin())) : IteratorT(iterator(points_.begin()));
  }

  template <typename IteratorT = const_iterator>
  IteratorT end(bool inverted) const noexcept {
    return inverted ? IteratorT(const_iterator(points_.rend())) : IteratorT(const_iterator(points_.end()));
  }
  template <typename IteratorT = iterator>
  IteratorT end(bool inverted) noexcept {
    return inverted ? IteratorT(iterator(points_.rend())) : IteratorT(iterator(points_.end()));
  }
  auto size() const noexcept { return points_.size(); }
  auto empty() const noexcept { return points_.empty(); }
  const ConstPoint3d& at(bool inverted, size_t idx) const noexcept {
    return inverted ? points_[points_.size() - 1 - idx] : points_[idx];
  }
  const ConstPoint3d& front(bool inverted) const noexcept { return inverted ? points_.back() : points_.front(); }
  const ConstPoint3d& back(bool inverted) const noexcept { return inverted ? points_.front() : points_.back(); }

 private:
  Points3d points_;
};  // class LineStringData

//! @defgroup LineStringPrimitives LineString
//! @ingroup Primitives
//!
//! ## General
//! LineStrings are a series of line segments defined by an ordered list of points.
//! A LineString consists at least of one point and must not contain the same point
//! multiple times in succession.
//!
//! LineStrings exist in const/non-const, 2d/3d and in normal/hybrid versions.
//! They all have the same interface but differ in the point types they provide.
//! While the normal versions return Point2d or Point3d as points,
//! the hybrid versions only return Eigen points when iterating.
//!
//! ## Design
//! The interface of linestrings is designed to look as similar to an
//! std::vector as possible. Therefore it is possible to pass Linestrings to
//! most stl-algorithms. Ranged for-loops are possible as well.
//!
//! ## Inverting
//! Linestrings can be inverted without cost overhead. When you iterate over
//! their points, you iterate them in reversed order. When you modify inverted
//! LineStrings, the non-inverted copies will be affected as well.
//!
//! ## Compound versions
//! The CompoundLineString3d/CompoundLineString2d/... classes allow to combine
//! multiple line strings so that they behave like one. They have the same
//! interface like a single normal line strings, but internally access the data
//! of multiple. The can also be passed to most geometry functions.
//!
//! If two adjacent linestrings in a compound linestring share the same point,
//! one of them is left out. CompoundLineStrings are immutable.

//! Implementation template class that is common to all LineStrings.
template <typename PointT>
class ConstLineStringImpl : public ConstPrimitive<LineStringData> {
 public:
  // using declarations
  using BasicPointType = traits::BasicPointT<PointT>;
  using ConstPointType = traits::ConstPointT<PointT>;
  using PointType = ConstPointType;  //! linestring will return this point type
  using MutablePointType = traits::MutablePointT<PointT>;
  using Category = traits::LineStringTag;

  // NOLINTNEXTLINE
  using const_iterator = internal::SelectLsIteratorT<const ConstPointType>;
  using iterator = const_iterator;                             // NOLINT
  using value_type = ConstPointType;                           // NOLINT
  using size_type = size_t;                                    // NOLINT
  using difference_type = typename iterator::difference_type;  // NOLINT

  using BasicIterator = internal::TransformIterator<const_iterator, const BasicPointType>;
  using BasicLineString = internal::SelectBasicLineStringT<BasicPointType>;
  using SegmentType = Segment<PointT>;
  using ConstSegmentType = traits::ConstPrimitiveType<SegmentType>;
  static constexpr traits::Dimensions Dimension = traits::PointTraits<PointT>::Dimension;

  //! Constructs a LineString or similar from an Id and a list of points
  explicit ConstLineStringImpl(Id id = InvalId, Points3d points = Points3d(),
                               const AttributeMap& attributes = AttributeMap())
      : ConstPrimitive{std::make_shared<LineStringData>(id, std::move(points), attributes)} {}

  //! Constructs a linestring from the data object of another linestring
  explicit ConstLineStringImpl(const std::shared_ptr<const LineStringData>& data, bool inverted = false)
      : ConstPrimitive(data), inverted_{inverted} {}

  //! construct from other ConstLineStrings
  template <typename OtherT>
  explicit ConstLineStringImpl(const ConstLineStringImpl<OtherT>& other)
      : ConstPrimitive(other.constData()), inverted_{other.inverted()} {}

  //! Returns whether this is an inverted linestring
  bool inverted() const noexcept { return inverted_; }

  //! Return the number of points in this linestring
  size_t size() const noexcept { return constData()->size(); }

  //! return if there are any points in this object
  bool empty() const noexcept { return constData()->empty(); }

  //! Returns an iterator to the start of the points
  /**
   * @see end, basicBegin
   * Dereferencing the iterator will convert the point from the internal
   * Point3d to PointType
   */
  const_iterator begin() const noexcept { return constData()->template begin<const_iterator>(inverted()); }
  //! Returns an iterator to end of the points
  /**
   * @see begin, basicEnd
   * Dereferencing the iterator will convert the point from the internal
   * Point3d to PointType
   */
  const_iterator end() const noexcept { return constData()->template end<const_iterator>(inverted()); }

  //! returns the first point (if it exist)
  /**
   * @see back, at
   * The internal point type is a different one, but because all points inherit
   * from the internal point type, no slicing is possible.
   */
  const ConstPointType& front() const noexcept { return constData()->front(inverted()); }

  /**
   * @brief returns the last point (if it exist)
   * @see front, operator[]
   *
   * The internal point type is a different one, but because all points inherit
   * from the internal point type, no slicing is possible.
   */
  const ConstPointType& back() const noexcept { return constData()->back(inverted()); }

  /**
   * @brief returns the point at this position
   * @see front, operator[]
   *
   * The internal point type is a different one, but because all points inherit
   * from the internal point type, no slicing is possible.
   */
  const ConstPointType& operator[](size_t idx) const noexcept {
    assert(idx < size());
    return constData()->at(inverted(), idx);
  }

  /**
   * @brief returns a normal iterator to the internal point type at begin
   * @see basicEnd, begin
   */
  BasicIterator basicBegin() const noexcept { return constData()->template begin<BasicIterator>(inverted_); }

  /**
   * @brief returns a normal iterator for the internal point type at end
   * @see basicBegin, end
   */
  BasicIterator basicEnd() const noexcept { return constData()->template end<BasicIterator>(inverted_); }

  /**
   * @brief returns the n-th segment. If n equals size() -1, the segment from
   * back() to front() is returned.
   */
  ConstSegmentType segment(size_t idx) const noexcept {
    assert(idx < size());
    auto first = begin() + idx;
    auto second = idx + 1 == size() ? begin() : first + 1;
    return ConstSegmentType(*first, *second);
  }

  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return size() <= 1 ? 0 : size() - 1; }

  /**
   * @brief Create a basic linestring (ie a vector of Eigen Points)
   *
   * A basic linestring is a simple vector with eigen points. You can do what
   * you want with these points, because changes to them will not affect lanelet
   * objects.
   *
   * BasicLineString is registered with boost::geometry, therefore you can use
   * it for geometry calculations.
   */
  BasicLineString basicLineString() const noexcept { return {basicBegin(), basicEnd()}; }

  template <typename LineStringT>
  class CompoundLineString;

 protected:
  // This class is only an implementation class and should never be instanciated
  ConstLineStringImpl(ConstLineStringImpl&& rhs) noexcept = default;
  ConstLineStringImpl& operator=(ConstLineStringImpl&& rhs) noexcept = default;
  ConstLineStringImpl(const ConstLineStringImpl& rhs) = default;
  ConstLineStringImpl& operator=(const ConstLineStringImpl& rhs) = default;
  ~ConstLineStringImpl() noexcept = default;

 private:
  bool inverted_{false};
};  // class ConstLineStringImpl

//! Implementation template class that is common to all non-const types
template <typename ConstLineStringT>
class LineStringImpl : public Primitive<ConstLineStringT> {
 protected:
  using Base = Primitive<ConstLineStringT>;
  using Base::data;

 public:
  using PointType = typename Base::MutablePointType;
  using iterator = internal::SelectLsIteratorT<PointType>;  // NOLINT
  using Base::Base;
  using Base::inverted;
  using Base::size;
  using SegmentType = typename Base::SegmentType;
  LineStringImpl() = default;  // Not inherited from base class

  //! Construct from other (mutable!) LineStrings
  template <typename OtherT>
  explicit LineStringImpl(const LineStringImpl<OtherT>& other) : Base(other) {}

  explicit LineStringImpl(const std::shared_ptr<const LineStringData>&, bool inverted) = delete;

  //! Construct from linestring data
  explicit LineStringImpl(const std::shared_ptr<LineStringData>& data, bool inverted) : Base(data, inverted) {}

  //! Copy assign from a normal vector. The Id of this object is unchanged.
  LineStringImpl& operator=(std::vector<PointType> rhs) {
    if (inverted()) {
      points() = utils::transform(rhs.rbegin(), rhs.rend(), [](const auto& v) { return Point3d(v); });
    } else {
      points() = utils::transform(rhs.begin(), rhs.end(), [](const auto& v) { return Point3d(v); });
    }
    return *this;
  }

  //! Move assign from a normal vector. Id and attributes stay unchanged.
  LineStringImpl& operator=(std::vector<Point3d>&& rhs) {
    if (inverted()) {
      utils::invert(rhs);
    }
    points() = std::move(rhs);
    return *this;
  }

  /**
   * @brief Inserts an element at a specific position
   *
   * Just as you would expect from an std::vector...
   * If you plan to insert more elements than one, use the ranged version below.
   * That saves you from some unnecessary cache regenerations.
   */
  iterator insert(iterator position, const PointType& point) {
    using Iter = LineStringData::iterator;
    auto fwIter = points().insert(internal::pointIter(position), Point3d(point));
    return inverted() ? Iter::reversed(fwIter + 1) : Iter(fwIter);
  }

  //! Inserts an range of elements at a specific position
  template <typename InIter>
  iterator insert(iterator position, InIter start, InIter end) {
    using Viter = typename internal::SelectInsertIterator<InIter>::Type;
    using RIter = LineStringData::iterator;
    if (inverted()) {
      using RViter = std::reverse_iterator<Viter>;
      auto fwIter = points().insert(internal::pointIter(position), RViter(end), RViter(start));
      return RIter(Points3d::reverse_iterator(fwIter + 1));
    }
    auto fwIter = points().insert(internal::pointIter(position), Viter(start), Viter(end));
    return RIter(fwIter);
  }

  //! inserts a new element at the end
  void push_back(const PointType& point) {  // NOLINT
    inverted() ? void(points().insert(points().begin(), Point3d(point))) : points().push_back(Point3d(point));
  }

  //! Removes point, returns iterator to the next point
  iterator erase(iterator position) {  // NOLINT
    if (inverted()) {
      position += 1;
    }
    using Iter = LineStringData::iterator;
    auto it = points().erase(internal::pointIter(position));
    return inverted() ? Iter::reversed(it) : Iter(it);
  }

  //! removes element from the end
  void pop_back() {  // NOLINT
    inverted() ? void(points().erase(points().begin())) : points().pop_back();
  }

  //! request a change in capacity
  void reserve(size_t num) { points().reserve(num); }

  //! Changes the number of points contained either by deleting them or by
  //! inserting new (invalId points).
  void resize(size_t num) {
    if (inverted()) {
      std::reverse(begin(), end());
    }
    points().resize(num);
    if (inverted()) {
      std::reverse(begin(), end());
    }
  }

  //! Remove all points, making the linestring invalid until new points are
  //! inserted.
  void clear() { points().clear(); }

  // inherit the const versions
  using Base::back;
  using Base::begin;
  using Base::end;
  using Base::front;
  using Base::operator[];
  using Base::segment;

  //! mutable iterator for the elements of this vector.
  iterator begin() { return inverted() ? iterator(points().rbegin()) : iterator(points().begin()); }

  //! mutable iterator to the end of the elements of this vector.
  iterator end() { return inverted() ? iterator(points().rend()) : iterator(points().end()); }

  //! get a refernce to the first element (make sure it exists)
  PointType& front() { return inverted() ? points().back() : points().front(); }

  //! get a reference to the last element if it exists
  PointType& back() { return inverted() ? points().front() : points().back(); }

  //! access element at specific position
  PointType& operator[](size_t idx) {
    assert(idx < size());
    return points()[inverted() ? points().size() - 1 - idx : idx];
  }

  /**
   * @brief returns the n-th segment. If n equals size() -1, the segment from
   * back() to front() is returned.
   */
  SegmentType segment(size_t idx) noexcept {
    assert(idx < size());
    const auto snd = idx + 1;
    return SegmentType(operator[](idx), operator[](snd == size() ? 0 : snd));
  }

 protected:
  // This class is only an implementation class and should never be instanciated
  LineStringImpl(LineStringImpl&& rhs) noexcept = default;
  LineStringImpl& operator=(LineStringImpl&& rhs) noexcept = default;
  LineStringImpl(const LineStringImpl& rhs) = default;
  LineStringImpl& operator=(const LineStringImpl& rhs) = default;
  ~LineStringImpl() noexcept = default;
  Points3d& points() { return data()->points(); }
};  // class LineStringImpl

//! @brief A normal 3d linestring with immutable data
//! @ingroup LineStringPrimitives
//! @ingroup ConstPrimitives
class ConstLineString3d : public ConstLineStringImpl<Point3d> {
 public:
  using ConstLineStringImpl::ConstLineStringImpl;
  using ConstType = ConstLineString3d;
  using MutableType = LineString3d;
  using TwoDType = ConstLineString2d;
  using ThreeDType = ConstLineString3d;
  using HybridType = ConstHybridLineString3d;
  ConstLineString3d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  ConstLineString3d invert() const noexcept { return ConstLineString3d{constData(), !inverted()}; }
};

//! @brief A normal 3d linestring with mutable data
//! @ingroup LineStringPrimitives
//! @ingroup Primitives
class LineString3d : public LineStringImpl<ConstLineString3d> {
 public:
  using LineStringImpl::LineStringImpl;
  using TwoDType = LineString2d;
  using ThreeDType = LineString3d;
  LineString3d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  LineString3d invert() const noexcept { return LineString3d{data(), !inverted()}; }
};

//! @brief A normal 2d linestring with immutable data
//! @ingroup LineStringPrimitives
//! @ingroup ConstPrimitives
class ConstLineString2d : public ConstLineStringImpl<Point2d> {
 public:
  using ConstLineStringImpl::ConstLineStringImpl;
  using ConstType = ConstLineString2d;
  using MutableType = LineString2d;
  using TwoDType = ConstLineString2d;
  using ThreeDType = ConstLineString3d;
  using HybridType = ConstHybridLineString2d;
  ConstLineString2d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  ConstLineString2d invert() const noexcept { return ConstLineString2d{constData(), !inverted()}; }
};

//! @brief A normal 2d linestring with mutable data
//! @ingroup LineStringPrimitives
//! @ingroup Primitives
class LineString2d : public LineStringImpl<ConstLineString2d> {
 public:
  using LineStringImpl::LineStringImpl;
  using TwoDType = LineString2d;
  using ThreeDType = LineString3d;
  LineString2d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  LineString2d invert() const noexcept { return LineString2d{data(), !inverted()}; }
};

//! @brief A Linestring that returns BasicPoint3d instead of Point3d
//!
//! For usage with boost::geometry. Has no mutable version.
//! @ingroup LineStringPrimitives
//! @ingroup ConstPrimitives
class ConstHybridLineString3d : public ConstLineString3d {
 public:
  using PointType = BasicPointType;
  using const_iterator = BasicIterator;  // NOLINT
  using iterator = BasicIterator;        // NOLINT
  using ConstType = ConstHybridLineString3d;
  using TwoDType = ConstHybridLineString2d;
  using ThreeDType = ConstHybridLineString3d;
  using SegmentType = Segment<BasicPointType>;

  using ConstLineString3d::ConstLineString3d;
  ConstHybridLineString3d() = default;  // Not inherited from base class
  explicit ConstHybridLineString3d(const ConstLineString3d& ls) : ConstLineString3d(ls) {}
  explicit ConstHybridLineString3d(const LineString3d& ls) : ConstLineString3d(ls) {}

  //! Returns an inverted linestring, O(0)
  ConstHybridLineString3d invert() const noexcept { return ConstHybridLineString3d{constData(), !inverted()}; }

  //! BasicPoint3d Iterator to begin
  BasicIterator begin() const noexcept { return basicBegin(); }

  //! BasicPoint3d Iterator to past-the-end
  BasicIterator end() const noexcept { return basicEnd(); }

  //! Get first BasicPoint3d
  const BasicPointType& front() const noexcept { return ConstLineString3d::front().basicPoint(); }

  //! Get last BasicPoint3d
  const BasicPointType& back() const noexcept { return ConstLineString3d::back().basicPoint(); }

  //! access BasicPoint3d at specific position
  const BasicPointType& operator[](size_t idx) const noexcept {
    return ConstLineString3d::operator[](idx).basicPoint();
  }

  /**
   * @brief returns the n-th segment. If n equals size() -1, the segment from
   * back() to front() is returned.
   */
  SegmentType segment(size_t idx) const noexcept {
    assert(idx < size());
    const auto snd = idx + 1;
    return {operator[](idx), operator[](snd == size() ? 0 : snd)};
  }
};

//! @brief A Linestring that returns BasicPoint2d instead of Point2d
//!
//! For usage with boost::geometry. Has no mutable version.
//! @ingroup LineStringPrimitives
//! @ingroup ConstPrimitives
class ConstHybridLineString2d : public ConstLineString2d {
 public:
  using PointType = BasicPointType;
  using const_iterator = BasicIterator;  // NOLINT
  using iterator = BasicIterator;        // NOLINT
  using ConstType = ConstHybridLineString2d;
  using TwoDType = ConstHybridLineString2d;
  using ThreeDType = ConstHybridLineString3d;
  using SegmentType = Segment<BasicPointType>;

  using ConstLineString2d::ConstLineString2d;
  ConstHybridLineString2d() = default;  // Not inherited from base class
  explicit ConstHybridLineString2d(const ConstLineString2d& ls) : ConstLineString2d(ls) {}
  explicit ConstHybridLineString2d(const LineString2d& ls) : ConstLineString2d(ls) {}

  //! Returns an inverted linestring, O(0)
  ConstHybridLineString2d invert() const noexcept { return ConstHybridLineString2d{constData(), !inverted()}; }

  //! BasicPoint2d Iterator to begin
  BasicIterator begin() const noexcept { return basicBegin(); }

  //! BasicPoint2d Iterator to past-the-end
  BasicIterator end() const noexcept { return basicEnd(); }

  //! Get first BasicPoint2d
  const BasicPointType& front() const noexcept { return ConstLineString2d::front().basicPoint(); }

  //! Get last BasicPoint2d
  const BasicPointType& back() const noexcept { return ConstLineString2d::back().basicPoint(); }

  //! access element at specific position
  const BasicPointType& operator[](size_t idx) const noexcept {
    return ConstLineString2d::operator[](idx).basicPoint();
  }

  /**
   * @brief returns the n-th segment. If n equals size() -1, the segment from
   * back() to front() is returned.
   */
  SegmentType segment(size_t idx) const noexcept {
    assert(idx < size());
    const auto snd = idx + 1;
    return {operator[](idx), operator[](snd == size() ? 0 : snd)};
  }
};

inline std::ostream& operator<<(std::ostream& stream, const ConstLineString2d& obj) {
  stream << "[id: " << obj.id();
  if (obj.inverted()) {
    stream << ", inverted";
  }
  stream << " point ids: ";
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    stream << it->id();
    if (std::next(it) != obj.end()) {
      stream << ", ";
    }
  }
  return stream << "]";
}

inline std::ostream& operator<<(std::ostream& stream, const ConstLineString3d& obj) {
  stream << "[id: " << obj.id();
  if (obj.inverted()) {
    stream << ", inverted";
  }
  stream << " point ids: ";
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    stream << it->id();
    if (std::next(it) != obj.end()) {
      stream << ", ";
    }
  }
  return stream << "]";
}

// comparison
template <typename LhsPointT, typename RhsPointT>
bool operator==(const ConstLineStringImpl<LhsPointT>& lhs, const ConstLineStringImpl<RhsPointT>& rhs) {
  return lhs.constData() == rhs.constData() && lhs.inverted() == rhs.inverted();
}
template <typename LhsPointT, typename RhsPointT>
bool operator!=(const ConstLineStringImpl<LhsPointT>& lhs, const ConstLineStringImpl<RhsPointT>& rhs) {
  return !(lhs == rhs);
}

template <typename PointT>
bool operator==(const ConstLineStringImpl<PointT>& lhs, const std::vector<PointT>& rhs) {
  return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

template <typename PointT>
bool operator==(const std::vector<PointT>& lhs, const ConstLineStringImpl<PointT>& rhs) {
  return rhs == lhs;
}

template <typename PointT>
bool operator!=(const ConstLineStringImpl<PointT>& lhs, const std::vector<PointT>& rhs) {
  return !(lhs == rhs);
}

template <typename PointT>
bool operator!=(const std::vector<PointT>& lhs, const ConstLineStringImpl<PointT>& rhs) {
  return !(lhs == rhs);
}

namespace traits {
template <typename T>
constexpr bool isLinestringT() {
  return isCategory<T, traits::LineStringTag>();
}

namespace detail {
template <typename T, typename Enable = void>
struct HybridType {};

template <typename T>
struct HybridType<T, std::enable_if_t<traits::isLinestringT<T>(), void>> {
  using Type = typename LineStringTraits<T>::HybridType;
};
template <typename T>
struct HybridType<T, std::enable_if_t<!traits::isLinestringT<T>(), void>> {
  using Type = typename LineStringTraits<T>::HybridType;
};
}  // namespace detail
template <typename LineStringT>
using HybridT = typename detail::HybridType<LineStringT>::Type;

template <typename LineStringT>
constexpr auto toHybrid(const LineStringT ls) {
  return HybridT<LineStringT>(ls);
}
}  // namespace traits

template <typename T, typename RetT>
using IfLS = std::enable_if_t<traits::isLinestringT<T>(), RetT>;

template <typename T1, typename T2, typename RetT>
using IfLS2 = std::enable_if_t<traits::isLinestringT<T1>() && traits::isLinestringT<T2>(), RetT>;

namespace utils {
using traits::toHybrid;

/**
 * @brief returns true if element of a lanelet primitive has a matching Id
 * @param ls the element holding other primitives
 * @param id id to look for
 * @return true if the primitive has such an element
 *
 * This function does not look for the id of the element, only its members
 * Works for linestrings and polylines.
 * A similar implementation exists for regulatory elements and lanelets.
 */
template <typename PointT>
bool has(const ConstLineStringImpl<PointT>& ls, Id id) {
  return std::any_of(ls.begin(), ls.end(), [&id](const auto& elem) { return elem.id() == id; });
}
}  // namespace utils

}  // namespace lanelet

// Hash function for usage in containers
namespace std {
template <>
struct hash<lanelet::LineString3d> : public lanelet::HashBase<lanelet::LineString3d> {};
template <>
struct hash<lanelet::ConstLineString3d> : public lanelet::HashBase<lanelet::ConstLineString3d> {};
template <>
struct hash<lanelet::LineString2d> : public lanelet::HashBase<lanelet::LineString2d> {};
template <>
struct hash<lanelet::ConstLineString2d> : public lanelet::HashBase<lanelet::ConstLineString2d> {};
template <>
struct hash<lanelet::ConstHybridLineString2d> : public lanelet::HashBase<lanelet::ConstHybridLineString2d> {};
template <>
struct hash<lanelet::ConstHybridLineString3d> : public lanelet::HashBase<lanelet::ConstHybridLineString3d> {};
}  // namespace std
