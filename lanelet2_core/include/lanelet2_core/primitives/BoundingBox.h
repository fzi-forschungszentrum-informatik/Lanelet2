#pragma once

#pragma GCC diagnostic push
#if defined __GNUC__ && (__GNUC__ >= 6)
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#endif
#include <Eigen/Geometry>

#include "lanelet2_core/primitives/Point.h"
#include "lanelet2_core/primitives/Traits.h"
#pragma GCC diagnostic pop

namespace lanelet {

/**
 * @brief Axis-Aligned bounding box in 2d
 *
 * Eigens aligned bounding box does not work for us in 2d, because of its memory alignment reqiurements.
 * This is basically the same implementation, but without the memory alginment. We don't have to do that for the 3d
 * case, because the 3d case does not use memory alignment in Eigen.
 */
class BoundingBox2d {
 public:
  enum { AmbientDimAtCompileTime = 2 };
  using Scalar = double;
  using ScalarTraits = Eigen::NumTraits<Scalar>;
  using Index = Eigen::Index;
  using RealScalar = ScalarTraits::Real;
  using NonInteger = ScalarTraits::NonInteger;
  using VectorType = BasicPoint2d;

  /** Define constants to name the corners of a 1D, 2D or 3D axis aligned bounding box */
  enum CornerType {
    /** 1D names @{ */
    Min = 0,
    Max = 1,
    /** @} */

    /** Identifier for 2D corner @{ */
    BottomLeft = 0,
    BottomRight = 1,
    TopLeft = 2,
    TopRight = 3,
    /** @} */

    /** Identifier for 3D corner  @{ */
    BottomLeftFloor = 0,
    BottomRightFloor = 1,
    TopLeftFloor = 2,
    TopRightFloor = 3,
    BottomLeftCeil = 4,
    BottomRightCeil = 5,
    TopLeftCeil = 6,
    TopRightCeil = 7
    /** @} */
  };

  /** Default constructor initializing a null box. */
  inline BoundingBox2d() {
    if (AmbientDimAtCompileTime != Eigen::Dynamic) {
      setEmpty();
    }
  }

  /** Constructs a null box with \a _dim the dimension of the ambient space. */
  inline explicit BoundingBox2d(Index dim) : min_(dim), max_(dim) { setEmpty(); }

  /** Constructs a box with extremities \a _min and \a _max.
   * \warning If either component of \a _min is larger than the same component of \a _max, the constructed box is empty.
   */
  template <typename OtherVectorType1, typename OtherVectorType2>
  inline BoundingBox2d(OtherVectorType1 min, OtherVectorType2 max) : min_(std::move(min)), max_(std::move(max)) {}

  /** Constructs a box containing a single point \a p. */
  template <typename Derived>
  inline explicit BoundingBox2d(const Eigen::MatrixBase<Derived>& p) : min_(p), max_(min_) {}

  inline BoundingBox2d(const Eigen::AlignedBox<double, 2>& other) : min_(other.min()), max_(other.max()) {}  // NOLINT

  /** \returns the dimension in which the box holds */
  inline Index dim() const {
    return AmbientDimAtCompileTime == Eigen::Dynamic ? min_.size() : Index(AmbientDimAtCompileTime);
  }

  /** \deprecated use isEmpty() */
  inline bool isNull() const { return isEmpty(); }

  /** \deprecated use setEmpty() */
  inline void setNull() { setEmpty(); }

  /** \returns true if the box is empty.
   * \sa setEmpty */
  inline bool isEmpty() const { return (min_.array() > max_.array()).any(); }

  /** Makes \c *this an empty box.
   * \sa isEmpty */
  inline void setEmpty() {
    min_.setConstant(ScalarTraits::highest());
    max_.setConstant(ScalarTraits::lowest());
  }

  /** \returns the minimal corner */
  inline const VectorType&(min)() const { return min_; }
  /** \returns a non const reference to the minimal corner */
  inline VectorType&(min)() { return min_; }
  /** \returns the maximal corner */
  inline const VectorType&(max)() const { return max_; }
  /** \returns a non const reference to the maximal corner */
  inline VectorType&(max)() { return max_; }

  /** \returns the center of the box */
  inline BasicPoint2d center() const { return (min_ + max_) / 2; }

  /** \returns the lengths of the sides of the bounding box.
   * Note that this function does not get the same
   * result for integral or floating scalar types: see
   */
  inline BasicPoint2d sizes() const { return max_ - min_; }

  /** \returns the volume of the bounding box */
  inline Scalar volume() const { return sizes().prod(); }

  /** \returns an expression for the bounding box diagonal vector
   * if the length of the diagonal is needed: diagonal().norm()
   * will provide it.
   */
  inline BasicPoint2d diagonal() const { return sizes(); }

  /** \returns the vertex of the bounding box at the corner defined by
   * the corner-id corner. It works only for a 1D, 2D or 3D bounding box.
   * For 1D bounding boxes corners are named by 2 enum constants:
   * BottomLeft and BottomRight.
   * For 2D bounding boxes, corners are named by 4 enum constants:
   * BottomLeft, BottomRight, TopLeft, TopRight.
   * For 3D bounding boxes, the following names are added:
   * BottomLeftCeil, BottomRightCeil, TopLeftCeil, TopRightCeil.
   */
  inline VectorType corner(CornerType corner) const {
    VectorType res;

    Index mult = 1;
    for (Index d = 0; d < dim(); ++d) {
      if ((mult & corner) != 0) {
        res[d] = max_[d];
      } else {
        res[d] = min_[d];
      }
      mult *= 2;
    }
    return res;
  }

  /** \returns a random point inside the bounding box sampled with
   * a uniform distribution */
  inline VectorType sample() const { return Eigen::AlignedBox<double, 2>(min_, max_).sample(); }

  /** \returns true if the point \a p is inside the box \c *this. */
  template <typename Derived>
  inline bool contains(const Eigen::MatrixBase<Derived>& p) const {
    return Eigen::AlignedBox<double, 2>(min_, max_).contains(p);
  }

  /** \returns true if the box \a b is entirely inside the box \c *this. */
  inline bool contains(const BoundingBox2d& b) const {
    return (min_.array() <= (b.min)().array()).all() && ((b.max)().array() <= max_.array()).all();
  }

  /** \returns true if the box \a b is intersecting the box \c *this.
   * \sa intersection, clamp */
  inline bool intersects(const BoundingBox2d& b) const {
    return (min_.array() <= (b.max)().array()).all() && ((b.min)().array() <= max_.array()).all();
  }

  /** Extends \c *this such that it contains the point \a p and returns a reference to \c *this.
   * \sa extend(const BoundingBox&) */
  template <typename Derived>
  inline BoundingBox2d& extend(const Eigen::MatrixBase<Derived>& p) {
    auto pN(p.derived().eval());
    min_ = min_.cwiseMin(pN);
    max_ = max_.cwiseMax(pN);
    return *this;
  }

  /** Extends \c *this such that it contains the box \a b and returns a reference to \c *this.
   * \sa merged, extend(const MatrixBase&) */
  inline BoundingBox2d& extend(const BoundingBox2d& b) {
    min_ = min_.cwiseMin(b.min_);
    max_ = max_.cwiseMax(b.max_);
    return *this;
  }

  /** Clamps \c *this by the box \a b and returns a reference to \c *this.
   * \note If the boxes don't intersect, the resulting box is empty.
   * \sa intersection(), intersects() */
  inline BoundingBox2d& clamp(const BoundingBox2d& b) {
    min_ = min_.cwiseMax(b.min_);
    max_ = max_.cwiseMin(b.max_);
    return *this;
  }

  /** Returns an BoundingBox2d that is the intersection of \a b and \c *this
   * \note If the boxes don't intersect, the resulting box is empty.
   * \sa intersects(), clamp, contains()  */
  inline BoundingBox2d intersection(const BoundingBox2d& b) const {
    return {min_.cwiseMax(b.min_), max_.cwiseMin(b.max_)};
  }

  /** Returns an BoundingBox2d that is the union of \a b and \c *this.
   * \note Merging with an empty box may result in a box bigger than \c *this.
   * \sa extend(const BoundingBox2d&) */
  inline BoundingBox2d merged(const BoundingBox2d& b) const { return {min_.cwiseMin(b.min_), max_.cwiseMax(b.max_)}; }

  /** Translate \c *this by the vector \a t and returns a reference to \c *this. */
  template <typename Derived>
  inline BoundingBox2d& translate(const Eigen::MatrixBase<Derived>& aT) {
    const auto t(aT.derived().eval());
    min_ += t;
    max_ += t;
    return *this;
  }

  /** \returns the squared distance between the point \a p and the box \c *this,
   * and zero if \a p is inside the box.
   * \sa exteriorDistance(const MatrixBase&), squaredExteriorDistance(const BoundingBox2d&)
   */
  template <typename Derived>
  inline Scalar squaredExteriorDistance(const Eigen::MatrixBase<Derived>& p) const {
    return Eigen::AlignedBox<double, 2>(min_, max_).squaredExteriorDistance(p);
  }

  /** \returns the squared distance between the boxes \a b and \c *this,
   * and zero if the boxes intersect.
   * \sa exteriorDistance(const BoundingBox2d&), squaredExteriorDistance(const MatrixBase&)
   */
  inline Scalar squaredExteriorDistance(const BoundingBox2d& b) const {
    using EigenBox = Eigen::AlignedBox<double, 2>;
    return EigenBox(min_, max_).squaredExteriorDistance(EigenBox(b.min_, b.max_));
  }

  /** \returns the distance between the point \a p and the box \c *this,
   * and zero if \a p is inside the box.
   * \sa squaredExteriorDistance(const MatrixBase&), exteriorDistance(const BoundingBox2d&)
   */
  template <typename Derived>
  inline NonInteger exteriorDistance(const Eigen::MatrixBase<Derived>& p) const {
    return std::sqrt(NonInteger(squaredExteriorDistance(p)));
  }

  /** \returns the distance between the boxes \a b and \c *this,
   * and zero if the boxes intersect.
   * \sa squaredExteriorDistance(const BoundingBox2d&), exteriorDistance(const MatrixBase&)
   */
  inline NonInteger exteriorDistance(const BoundingBox2d& b) const {
    using std::sqrt;
    return sqrt(NonInteger(squaredExteriorDistance(b)));
  }

  /** Copy constructor with scalar type conversion */
  template <typename OtherScalarType>
  inline explicit BoundingBox2d(const Eigen::AlignedBox<OtherScalarType, AmbientDimAtCompileTime>& other) {
    min_ = (other.min)().template cast<Scalar>();
    max_ = (other.max)().template cast<Scalar>();
  }

  /** \returns \c true if \c *this is approximately equal to \a other, within the precision
   * determined by \a prec.
   *
   * \sa MatrixBase::isApprox() */
  bool isApprox(const BoundingBox2d& other, const RealScalar& prec = ScalarTraits::dummy_precision()) const {
    return min_.isApprox(other.min_, prec) && max_.isApprox(other.max_, prec);
  }

 private:
  VectorType min_, max_;
};

/**
 * @brief Convenience type for an axis aligned bounding box in 3d.
 *
 * Can be used as Eigen::AlignedBox2d and as boost::geometry::Box.
 */
using BoundingBox3d = Eigen::AlignedBox3d;

namespace traits {
template <>
struct PrimitiveTraits<BoundingBox2d> {
  using ConstType = BoundingBox2d;
  using MutableType = BoundingBox2d;
  using TwoDType = BoundingBox2d;
  using ThreeDType = BoundingBox3d;
  using Category = BoundingBoxTag;
};
template <>
struct PrimitiveTraits<BoundingBox3d> {
  using ConstType = BoundingBox3d;
  using MutableType = BoundingBox3d;
  using TwoDType = BoundingBox2d;
  using ThreeDType = BoundingBox3d;
  using Category = BoundingBoxTag;
};

inline BoundingBox3d to3D(const BoundingBox2d& primitive) { return {to3D(primitive.min()), to3D(primitive.max())}; }
inline BoundingBox2d to2D(const BoundingBox3d& primitive) { return {to2D(primitive.min()), to2D(primitive.max())}; }
}  // namespace traits
}  // namespace lanelet
