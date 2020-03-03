#pragma once
#include <boost/geometry/strategies/cartesian/distance_projected_point.hpp>
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

template <typename InPointT, typename OutPointT>
std::enable_if_t<std::is_same<InPointT, OutPointT>::value> convertOrAssign(const InPointT& pIn, OutPointT& pOut) {
  pOut = pIn;
}

template <typename InPointT, typename OutPointT>
std::enable_if_t<!std::is_same<InPointT, OutPointT>::value> convertOrAssign(const InPointT& pIn, OutPointT& pOut) {
  boost::geometry::convert(pIn, pOut);
}

/**
 * @brief A strategy for boost::distance.
 *
 * Works just like the default strategy for boost::distance on linestrings, but
 * additionally stores some information on the closest segment.
 */
template <typename ProjPoint>
class ProjectedPoint : public boost::geometry::strategy::distance::projected_point<> {
 public:
  using Strategy = boost::geometry::strategy::distance::pythagoras<void>;
  template <typename Point1T, typename Point2T, typename CalculationResult>
  auto updateClosestPoint(const Point1T& pSeg1, const Point1T& pSeg2, Point2T& pProj, CalculationResult d) const {
    using namespace boost::geometry;
    if (result->dMin < 0 || d < result->dMin) {
      convert(pProj, result->projectedPoint);
      convertOrAssign(pSeg1, result->segmentPoint1);
      convertOrAssign(pSeg2, result->segmentPoint2);
      result->dMin = d;
    }
    return d;
  }
  template <typename Point, typename PointOfSegment>
  inline typename calculation_type<Point, PointOfSegment>::type apply(Point const& p, PointOfSegment const& p1,
                                                                      PointOfSegment const& p2) const {
    using namespace boost::geometry;
    assert_dimension_equal<Point, PointOfSegment>();

    using CalculationResult = typename calculation_type<Point, PointOfSegment>::type;

    // A projected point of points in Integer coordinates must be able to be
    // represented in FP.
    using FpVector =  // NOLINT
        model::point<CalculationResult, dimension<PointOfSegment>::value,
                     typename coordinate_system<PointOfSegment>::type>;

    FpVector v;
    FpVector w;
    FpVector projected;

    convert(p2, v);
    convert(p, w);
    convert(p1, projected);
    subtract_point(v, projected);
    subtract_point(w, projected);

    Strategy strategy;
    boost::ignore_unused_variable_warning(strategy);

    auto zero = CalculationResult{};
    CalculationResult const c1 = dot_product(w, v);
    if (c1 <= zero) {
      return updateClosestPoint(p1, p2, p1, strategy.apply(p, p1));
    }
    CalculationResult const c2 = dot_product(v, v);
    if (c2 <= c1) {
      return updateClosestPoint(p1, p2, p2, strategy.apply(p, p2));
    }

    // See above, c1 > 0 AND c2 > c1 so: c2 != 0
    CalculationResult const b = c1 / c2;

    multiply_value(v, b);
    add_point(projected, v);
    return updateClosestPoint(p1, p2, projected, strategy.apply(p, projected));
  }
  struct Result {
    ProjPoint projectedPoint, segmentPoint1, segmentPoint2;
    double dMin{-1.};
  };
  mutable std::shared_ptr<Result> result{std::make_shared<Result>()};
};
}  // namespace helper
}  // namespace lanelet

// Helper function definitions for our strategy.
namespace boost {
namespace geometry {
namespace strategy {
namespace distance {
namespace services {

template <typename ProjPoint>
struct tag<lanelet::helper::ProjectedPoint<ProjPoint>> {
  typedef strategy_tag_distance_point_segment type;  // NOLINT
};

template <typename ProjPoint, typename P, typename PS>
struct return_type<lanelet::helper::ProjectedPoint<ProjPoint>, P, PS>
    : lanelet::helper::ProjectedPoint<ProjPoint>::template calculation_type<P, PS> {};

template <typename ProjPoint>
struct comparable_type<lanelet::helper::ProjectedPoint<ProjPoint>> {
  // Define a projected_point strategy with its underlying point-point-strategy
  // being comparable
  typedef lanelet::helper::ProjectedPoint<ProjPoint> type;  // NOLINT
};

template <typename ProjPoint>
struct get_comparable<lanelet::helper::ProjectedPoint<ProjPoint>> {
  using comparable_type =  // NOLINT
      typename comparable_type<lanelet::helper::ProjectedPoint<ProjPoint>>::type;

 public:
  static inline comparable_type apply(lanelet::helper::ProjectedPoint<ProjPoint> const& p) {  // NOLINT
    return p;
  }
};

template <typename ProjPoint, typename P, typename PS>
struct result_from_distance<lanelet::helper::ProjectedPoint<ProjPoint>, P, PS> {
 private:
  using return_type =  // NOLINT
      typename return_type<lanelet::helper::ProjectedPoint<ProjPoint>, P, PS>::type;

 public:
  template <typename T>
  static inline return_type apply(lanelet::helper::ProjectedPoint<ProjPoint> const&,  // NOLINT
                                  T const& value) {
    typename lanelet::helper::ProjectedPoint<ProjPoint>::Strategy s;
    return result_from_distance<typename lanelet::helper::ProjectedPoint<ProjPoint>::Strategy, P, PS>::apply(s, value);
  }
};
}  // namespace services
}  // namespace distance
}  // namespace strategy
}  // namespace geometry
}  // namespace boost
