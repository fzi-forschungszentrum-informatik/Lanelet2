#pragma once
#include <boost/noncopyable.hpp>
#include <utility>

#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/utility/CompoundIterator.h"

namespace lanelet {
namespace internal {
/**
 * @brief This class selects which iterator CompoundLineStringImpl needs.
 */
template <typename PointT>
struct SelectCompoundLsIterator {
  using BaseIterator = internal::UniqueCompoundIterator<const ConstLineStrings3d>;
  using Iterator = internal::TransformIterator<internal::ReverseAndForwardIterator<BaseIterator>, const PointT>;
};

template <>
struct SelectCompoundLsIterator<ConstPoint3d> {
  using BaseIterator = internal::UniqueCompoundIterator<const ConstLineStrings3d>;
  using Iterator = internal::ReverseAndForwardIterator<BaseIterator>;
};

template <typename T>
using SelectCompoundLsIteratorT = typename SelectCompoundLsIterator<T>::Iterator;
}  // namespace internal

/**
 * @brief Common data object for all CompoundLineStrings
 *
 * This object is only for internal data management and should not be used
 * directly.
 *
 * CompoundLineStrings merely have a shared_ptr to this object and forward all
 * calls to this object.
 */
class CompoundLineStringData : boost::noncopyable {
  using BaseIter = typename internal::SelectCompoundLsIterator<ConstPoint3d>::BaseIterator;
  using Reverse = std::reverse_iterator<BaseIter>;

 public:
  using RFIter = internal::SelectCompoundLsIteratorT<ConstPoint3d>;
  explicit CompoundLineStringData(ConstLineStrings3d lineStrings) : ls_{std::move(lineStrings)} {}
  ConstPoints3d points() const { return ConstPoints3d(BaseIter::begin(ls_), BaseIter::end(ls_)); }
  const ConstLineStrings3d& lineStrings() const { return ls_; }
  RFIter pointsBegin(bool inverted) const {
    return inverted ? RFIter(Reverse(BaseIter::end(ls_))) : RFIter(BaseIter::begin(ls_));
  }
  RFIter pointsEnd(bool inverted) const {
    return inverted ? RFIter(Reverse(BaseIter::begin(ls_))) : RFIter(BaseIter::end(ls_));
  }

 private:
  const ConstLineStrings3d ls_;  // NOLINT
};

/**
 * @brief A collection of lineStrings that act as one line string
 * @tparam PointT the point type that the linestring returns
 *
 * A CompoundLineString has the same interface as a LineString2d/LineString3d,
 * but internally accesses the points of multiple linestrings. Because it just
 * provides an interface to the points, the internal data can not be modified.
 * If adjacent linestrings share the same points, one of them is discarded.
 *
 */
template <typename PointT>
class CompoundLineStringImpl {
  using RFIter = CompoundLineStringData::RFIter;

 public:
  // using declarations
  using DataType = CompoundLineStringData;
  using PointType = PointT;
  using BasicPointType = traits::BasicPointT<PointT>;
  using ConstPointType = traits::ConstPointT<PointT>;
  using Category = traits::LineStringTag;
  static constexpr traits::Dimensions Dimension = traits::PointTraits<PointT>::Dimension;
  using BasicIterator = internal::TransformIterator<RFIter, const BasicPointType>;
  using const_iterator =  // NOLINT
      internal::SelectCompoundLsIteratorT<ConstPointType>;
  using iterator = const_iterator;  // NOLINT
  using BasicLineString = internal::SelectBasicLineStringT<BasicPointType>;
  using SegmentType = Segment<ConstPointType>;
  /**
   * @brief Construct from a vector of ConstLineString3d
   * @param ls objects to construct from. The order will also be the order of
   * the linestrings.
   */
  explicit CompoundLineStringImpl(const ConstLineStrings3d& ls = ConstLineStrings3d())
      : data_{std::make_shared<CompoundLineStringData>(ls)} {}

  //! Construct from a vector of LineStrings or CompoundLineStrings
  template <typename OtherT, typename = IfLS<OtherT, void>>
  explicit CompoundLineStringImpl(const std::vector<OtherT>& lss)
      : data_{std::make_shared<CompoundLineStringData>(
            utils::transform(lss, [](const auto& e) { return traits::to3D(traits::toConst(e)); }))} {}

  //! Internal construction from data pointer
  CompoundLineStringImpl(CompoundLineStringDataConstPtr data, bool inverted)
      : data_(std::move(data)), inverted_{inverted} {}

  //! construct from other CompoundLineString
  template <typename OtherT>
  explicit CompoundLineStringImpl(const CompoundLineStringImpl<OtherT>& other)
      : data_(other.constData()), inverted_{other.inverted()} {}

  // default the rest (constructor above deletes copy constructors)
  CompoundLineStringImpl(CompoundLineStringImpl&& rhs) noexcept = default;
  CompoundLineStringImpl& operator=(CompoundLineStringImpl&& rhs) noexcept = default;
  CompoundLineStringImpl(const CompoundLineStringImpl& rhs) = default;
  CompoundLineStringImpl& operator=(const CompoundLineStringImpl& rhs) = default;
  ~CompoundLineStringImpl() noexcept = default;

  /**
   * @brief returns whether this is an inverted CompoundLineString
   *
   * Inverted linestrings behave just as normal linestrings but provide access
   * to the points in inverted order.
   */
  bool inverted() const noexcept { return inverted_; }

  /**
   * @brief return the total number of unique points
   *
   * Unique means that adjacent points with the same id are not counted.
   * Note that this O(n) in the number of points, not O(0).
   */
  size_t size() const noexcept { return std::distance(begin(), end()); }

  /**
   * @brief return whether this contains any points
   *
   * Note this is O(n) in the number of points.
   */
  bool empty() const noexcept { return size() == 0; }

  /**
   * @brief returns an iterator to the start of the points
   * @see end, basicBegin
   *
   * Dereferencing the iterator will convert the point from the internal
   * Point3d to PointT. It will return the first point of the first
   * linestring.
   */
  const_iterator begin() const noexcept { return constData()->pointsBegin(inverted()); }
  /**
   * @brief returns an iterator to end of the points
   * @see begin, basicEnd
   * Dereferencing the iterator will convert the point from the internal
   * Point3d to PointType
   */
  const_iterator end() const noexcept { return constData()->pointsEnd(inverted()); }

  /**
   * @brief returns the first point of the first linestring
   * @see back, at
   * The internal point type is a different one, but because all points
   * inherit from the internal point type, no slicing is possible. It will
   * return the first point of the first linestring.
   */
  const PointType& front() const noexcept { return *begin(); }

  /**
   * @brief returns the last point of the last linestring
   * @see front, at
   *
   * The internal point type is a different one, but because all points
   * inherit from the internal point type, no slicing is possible.
   */
  const PointType& back() const noexcept { return *std::prev(end()); }

  /**
   * @brief returns the point at this position
   * @see front
   *
   * The internal point type is a different one, but because all points
   * inherit from the internal point type, no slicing is possible.
   *
   * Note this is O(n) in the number of points, not O(0)
   */
  const PointType& operator[](size_t idx) const {
    assert(idx < size());
    return *std::next(begin(), idx);
  }

  /**
   * @brief returns the n-th segment. If n equals size() -1, the segment from
   * back() to front() is returned.
   *
   * It is not a good idea to use this in a loop, because this has O(n) for
   * every iteration. Use helper::forEachSegment instead.
   */
  SegmentType segment(size_t idx) const noexcept {
    assert(idx < size());
    const auto first = std::next(begin(), idx);
    const auto second = idx + 1 == size() ? begin() : std::next(first);
    return SegmentType(*first, *second);
  }

  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return std::max(size_t(1), size()) - 1; }

  /**
   * @brief returns a normal iterator to the internal point type at begin
   * @see basicEnd, begin
   */
  BasicIterator basicBegin() const { return constData()->pointsBegin(inverted()); }

  /**
   * @brief returns a normal iterator for the internal point type at end
   * @see basicBegin, end
   */
  BasicIterator basicEnd() const { return constData()->pointsEnd(inverted()); }
  /**
   * @brief create a basic linestring from this linestring
   * @return the basic linestring
   *
   * A basic linestring is a simple vector with eigen points. You can do what
   * you want with these points, because changes to them will not affect
   * lanelet objects.
   *
   * BasicLineString is registered with boost::geometry, therefore you can use
   * it for geometry calculations.
   */
  BasicLineString basicLineString() const { return {basicBegin(), basicEnd()}; }

  /**
   * @brief returns the internal data on the linestrings managed by this
   * object
   */
  std::shared_ptr<const CompoundLineStringData> constData() const noexcept { return data_; }

  ConstLineStrings3d lineStrings() const {
    return inverted()
               ? utils::invert(utils::transform(data_->lineStrings(), [](const auto& ls) { return ls.invert(); }))
               : data_->lineStrings();
  }

  /**
   * @brief returns the ids of all linestrings in order
   * @return list of ids
   */
  Ids ids() const {
    auto ids = utils::transform(data_->lineStrings(), [](const auto& ls) { return ls.id(); });
    if (inverted()) {
      return utils::invert(ids);
    }
    return ids;
  }

 private:
  CompoundLineStringDataConstPtr data_;
  bool inverted_{false};
};

/**
 * @brief A Compound linestring in 2d (returns Point2d)
 * @ingroup LineStringPrimitives
 * @see CompoundLineStringImpl
 */
class CompoundLineString2d : public CompoundLineStringImpl<ConstPoint2d> {
 public:
  using TwoDType = CompoundLineString2d;
  using ThreeDType = CompoundLineString3d;
  using HybridType = CompoundHybridLineString2d;
  using ConstType = CompoundLineString2d;
  using MutableType = void;
  using CompoundLineStringImpl::CompoundLineStringImpl;

  explicit CompoundLineString2d(const CompoundLineStrings2d& other)
      : CompoundLineString2d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  CompoundLineString2d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  CompoundLineString2d invert() const noexcept { return CompoundLineString2d{constData(), !inverted()}; }
};

/**
 * @brief A Compound linestring in 3d (returns Point3d)
 * @ingroup LineStringPrimitives
 * @see CompoundLineStringImpl
 */
class CompoundLineString3d : public CompoundLineStringImpl<ConstPoint3d> {
 public:
  using TwoDType = CompoundLineString2d;
  using ThreeDType = CompoundLineString3d;
  using HybridType = CompoundHybridLineString3d;
  using ConstType = CompoundLineString3d;
  using MutableType = void;
  using CompoundLineStringImpl::CompoundLineStringImpl;
  explicit CompoundLineString3d(const CompoundLineStrings3d& other)
      : CompoundLineString3d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  CompoundLineString3d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  CompoundLineString3d invert() const noexcept { return CompoundLineString3d{constData(), !inverted()}; }
};

/**
 * @brief A hybrid compound linestring in 2d (returns BasicPoint2d)
 * @ingroup LineStringPrimitives
 * @see CompoundLineStringImpl
 */
class CompoundHybridLineString2d : public CompoundLineStringImpl<BasicPoint2d> {
 public:
  using TwoDType = CompoundHybridLineString2d;
  using ThreeDType = CompoundHybridLineString3d;
  using HybridType = CompoundHybridLineString2d;
  using ConstType = CompoundHybridLineString2d;
  using MutableType = void;
  using CompoundLineStringImpl::CompoundLineStringImpl;
  explicit CompoundHybridLineString2d(const CompoundHybridLineStrings2d& other)
      : CompoundHybridLineString2d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  CompoundHybridLineString2d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  CompoundHybridLineString2d invert() const noexcept { return CompoundHybridLineString2d{constData(), !inverted()}; }
};

/**
 * @brief A hybrid compound linestring in 3d (returns BasicPoint3d)
 * @ingroup LineStringPrimitives
 * @see CompoundLineStringImpl
 */
class CompoundHybridLineString3d : public CompoundLineStringImpl<BasicPoint3d> {
 public:
  using TwoDType = CompoundHybridLineString2d;
  using ThreeDType = CompoundHybridLineString3d;
  using HybridType = CompoundHybridLineString3d;
  using ConstType = CompoundHybridLineString3d;
  using MutableType = void;
  using CompoundLineStringImpl::CompoundLineStringImpl;
  explicit CompoundHybridLineString3d(const CompoundHybridLineStrings3d& other)
      : CompoundHybridLineString3d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  CompoundHybridLineString3d() = default;  // Not inherited from base class

  //! create a new, inverted linestring from this one
  CompoundHybridLineString3d invert() const noexcept { return CompoundHybridLineString3d{constData(), !inverted()}; }
};

namespace utils {
/**
 * @brief returns true if element of a  primitive has a matching Id
 * @param ls the element holding other primitives
 * @param id id to look for
 * @return true if the primitive has such an element
 *
 * This function does not look for the id of the element, only its members
 * Works for linestrings and polylines.
 * A similar implementation exists for regulatory elements and lanelets.
 */
template <typename PointT>
bool has(const CompoundLineStringImpl<PointT>& ls, Id id) {
  return std::any_of(ls.begin(), ls.end(), [&id](const auto& elem) { return elem.id() == id; });
}
}  // namespace utils

template <typename Point1T, typename Point2T>
bool operator==(const CompoundLineStringImpl<Point1T>& lhs, const CompoundLineStringImpl<Point2T>& rhs) {
  return lhs.lineStrings() == rhs.lineStrings();
}

template <typename Point1T, typename Point2T>
bool operator!=(const CompoundLineStringImpl<Point1T>& lhs, const CompoundLineStringImpl<Point2T>& rhs) {
  return !(rhs == lhs);
}
template <typename Point1T, typename Point2T>
bool operator==(const CompoundLineStringImpl<Point1T>& lhs, const std::vector<Point2T>& rhs) {
  return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

template <typename Point1T, typename Point2T>
bool operator==(const std::vector<Point1T>& lhs, const CompoundLineStringImpl<Point2T>& rhs) {
  return rhs == lhs;
}

template <typename Point1T, typename Point2T>
bool operator!=(const CompoundLineStringImpl<Point1T>& lhs, const std::vector<Point2T>& rhs) {
  return !(lhs == rhs);
}

template <typename Point1T, typename Point2T>
bool operator!=(const std::vector<Point1T>& lhs, const CompoundLineStringImpl<Point2T>& rhs) {
  return !(lhs == rhs);
}

template <typename PointT>
inline std::ostream& operator<<(std::ostream& stream, const CompoundLineStringImpl<PointT>& obj) {
  auto ids = obj.ids();
  stream << "[";
  if (!ids.empty()) {
    stream << "ids: ";
    std::copy(ids.begin(), ids.end(), std::ostream_iterator<Id>(stream, " "));
  }
  if (obj.inverted()) {
    stream << ", inverted";
  }
  return stream << "]";
}
}  // namespace lanelet
