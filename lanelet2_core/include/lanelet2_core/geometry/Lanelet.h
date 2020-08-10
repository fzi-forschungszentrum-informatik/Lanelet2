#pragma once

#include "lanelet2_core/primitives/Lanelet.h"

namespace lanelet {
namespace geometry {
/**
 * @brief Checks whether a point is within or at the border of a lanelet
 * @param lanelet for the check
 * @param point point to check
 * @return true if the point is within or at the border, false otherwise
 */
template <typename LaneletT>
IfLL<LaneletT, bool> inside(const LaneletT& lanelet, const BasicPoint2d& point);

/**
 * @brief approximates length by sampling points along left bound
 *
 * avoids to calculate the centerline which might be expensive.
 */
template <typename LaneletT>
double approximatedLength2d(const LaneletT& lanelet);

/**
 * @brief calculate length of centerline in 2d
 * @param lanelet lanelet to do this for
 * @return length in m
 */
template <typename LaneletT>
double length2d(const LaneletT& lanelet);

/**
 * @brief calculate length of centerline in 3d
 * @param lanelet lanelet to do this for
 * @return length in m
 */
template <typename LaneletT>
double length3d(const LaneletT& lanelet);

/**
 * @brief calculates distance in 2d to the centerline of a lanelet.
 * @param lanelet lanelet to take the centerline from
 * @param point the point
 * @return metric distance
 *
 * Has linear complexity on the number of points in the lanelet.
 */
template <typename LaneletT>
double distanceToCenterline2d(const LaneletT& lanelet, const BasicPoint2d& point);

/**
 * @brief calculates distance in 3d to centerline of a lanelet.
 * @brief calculates distance in 2d to the centerline of a lanelet.
 * @param lanelet lanelet to take the centerline from
 * @param point the point
 * @return metric distance
 *
 * Unlike distance3d, the surface does not have to be planar.
 * Has linear complexity on the number of points in the lanelet.
 */
template <typename LaneletT>
double distanceToCenterline3d(const LaneletT& lanelet, const BasicPoint3d& point);

/**
 * @brief calculates an up-right 2d bounding box
 * @param lanelet lanelet to calculate it from
 * @return the bounding box.
 *
 * Linear on number of points.
 */
template <typename LaneletT>
IfLL<LaneletT, BoundingBox2d> boundingBox2d(const LaneletT& lanelet);

/**
 * @brief calculates 3d bounding box
 * @param lanelet lanelet to calculate it from.
 * @return the bounding box
 */
template <typename LaneletT>
IfLL<LaneletT, BoundingBox3d> boundingBox3d(const LaneletT& lanelet);

/**
 * @brief test whether two lanelets intersect in 2d.
 * @param lanelet lanelet to check for
 * @param otherLanelet other lanelet to check for
 * @see overlaps2d
 * @return true if lanelets have intersections.
 *
 * This also returns true if the two lanelets only touch each other. Use
 * overlaps if you do not want this.
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> intersects2d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet);

/**
 * @brief test whether two lanelets overlap in 2d.
 *
 * Returns true if the area shared by the two lanelets is >0.
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> overlaps2d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet);
/**
 * @brief test whether two lanelets intersect in 2d.
 * @param lanelet lanelet to check for
 * @param otherLanelet other lanelet to check for
 * @param heightTolerance distance in z below which lanelets are considered as
 * intersecting
 * @todo this is currently only an approximation based on the centerlines
 * @return true if lanelets have intersections.
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> intersects3d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet,
                                   double heightTolerance = 3.);

/**
 * @brief test whether two lanelets overlap in 3d.
 * @todo this is currently only an approximation based on the centerlines
 *
 * Returns true if the area shared by the two lanelets is >0.
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> overlaps3d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet, double heightTolerance = 3);
/**
 * @brief calculates points of intersection between centerlines in 2d.
 * @param lanelet first lanelet to calculate from
 * @param otherLanelet second lanelet to calculate from
 * @param distanceThis optional: if not null will contain distances to travel
 * along the centerline to each intersection point. Same size as the returned
 * result.
 * @param distanceOther optional: same for the other lanelet
 * @return vector of intersection points, of empty if none.
 */
template <typename Lanelet1T, typename Lanelet2T>
BasicPoints2d intersectCenterlines2d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet,
                                     std::vector<double>* distanceThis = nullptr,
                                     std::vector<double>* distanceOther = nullptr);

/**
 * @brief checks if a lanelet is direcly left of another by checking if they
 * share the same boundary
 *
 * Be aware that the orientation of the lanelets (see Lanelet::invert()) is
 * important
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, IfLL<Lanelet2T, bool>> leftOf(const Lanelet1T& left, const Lanelet2T& right);

/**
 * @brief checks if a lanelet is direcly right of another by checking if they
 * share the same boundary
 *
 * Be aware that the orientation of the lanelets (see Lanelet::invert()) is
 * important.
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, IfLL<Lanelet2T, bool>> rightOf(const Lanelet1T& right, const Lanelet2T& left);

/**
 * @brief checks if a lanelet is the direct successor by checking if they
 * share the same start/endpoints
 *
 * Be aware that the orientation of the lanelets (see Lanelet::invert()) is
 * important.
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, IfLL<Lanelet2T, bool>> follows(const Lanelet1T& prev, const Lanelet2T& next);

/**
 * @brief find a common line string in ll and other.
 * @param ll Lanelet
 * @param other Lanelet
 * @param allowInverted if true, the orientation of the line strings is ignored
 * @return line string in ll if it is shared with other
 */
template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, IfLL<Lanelet2T, Optional<ConstLineString3d>>> determineCommonLine(const Lanelet1T& ll,
                                                                                  const Lanelet2T& other,
                                                                                  bool allowInverted = false);

/**
 * @brief calculates the maximum velocity without exceding a maximum lateral
 * acceleration.
 * @param lanelet lanelet to calculate this from
 * @param position position in 2d next to the lanelet
 * @param maxLateralAcceleration the maximum desired acceleration
 * @return the maximum velocity. Can be Inf if the lanelet has no curvature
 * locally.
 */
template <typename LaneletT>
Velocity maxCurveSpeed(const LaneletT& lanelet, const BasicPoint2d& position,
                       const Acceleration& maxLateralAcceleration = 2.0 * units::MPS2());
}  // namespace geometry
}  // namespace lanelet

#include "impl/Lanelet.h"
