#pragma once

#include "../primitives/Area.h"
#include "../primitives/Lanelet.h"
#include "Polygon.h"

namespace lanelet {
namespace geometry {
/**
 * @brief Checks whether a point is within or at the border of a area
 * @param area for the check
 * @param point point to check
 * @return true if the point is within or at the border, false otherwise
 */
template <typename AreaT>
IfAr<AreaT, bool> inside(const AreaT& area, const BasicPoint2d& point);

/**
 * @brief calculates an up-right 2d bounding box
 * @param area area to calculate it from
 * @return the bounding box.
 *
 * Linear on number of points.
 */
template <typename AreaT>
IfAr<AreaT, BoundingBox2d> boundingBox2d(const AreaT& area);

/**
 * @brief calculates 3d bounding box
 * @param area area to calculate it from.
 * @return the bounding box
 */
template <typename AreaT>
IfAr<AreaT, BoundingBox3d> boundingBox3d(const AreaT& area);

//! test whether two areas intersect in 2d.
template <typename Area1T, typename Area2T>
IfAr<Area1T, bool> intersects2d(const Area1T& area, const Area2T& otherArea);

//! test whether two areas overlap in 2d (common area < 0).
//! This is an approximation that ignores the holes of the areas!
template <typename AreaT>
IfAr<AreaT, bool> overlaps2d(const AreaT& area, const AreaT& otherArea);

//! test whether two areas overlap in 3d.
//! This is an approximation that uses the overlap of the outer bound
template <typename AreaT>
IfAr<AreaT, bool> overlaps3d(const AreaT& area, const AreaT& otherArea, double heightTolerance);

//! test whether an area and a lanelet overlap in 2d
//! This is an approximation that uses the overlap of the outer bound
template <typename AreaT, typename LaneletT>
IfAr<AreaT, IfLL<LaneletT, bool>> overlaps2d(const AreaT& area, const LaneletT& lanelet);

//! test whether an area and a lanelet overlap in 3d
//! This is an approximation that uses the overlap of the outer bound
template <typename AreaT, typename LaneletT>
IfAr<AreaT, IfLL<LaneletT, bool>> overlaps3d(const AreaT& area, const LaneletT& lanelet, double heightTolerance);

//! Test whether area is left of lanelet
inline bool leftOf(const ConstLanelet& right, const ConstArea& left);

//! Test whether area is right of lanelet
inline bool rightOf(const ConstLanelet& left, const ConstArea& area);

//! Test whether area follows lanelet
inline bool follows(const ConstLanelet& prev, const ConstArea& next);

//! Test whether lanelet follows area
inline bool follows(const ConstArea& prev, const ConstLanelet& next);

//! Test if two areas are adjacent
inline bool adjacent(const ConstArea& area1, const ConstArea& area2);

/**
 * Find line string in area ar that borders Lanelet ll if ar follows ll
 * @param ll Lanelet
 * @param ar Area that is following ll
 * @return LineString3d if it exists
 */
inline Optional<ConstLineString3d> determineCommonLinePreceding(const ConstLanelet& ll, const ConstArea& ar);

/**
 * Find line string in area ar that borders Lanelet ll if ar precedes ll
 * @param ar Lanelet
 * @param ll Area that is preceding ll
 * @return LineString3d if it exists
 */
inline Optional<ConstLineString3d> determineCommonLineFollowing(const ConstArea& ar, const ConstLanelet& ll);

/**
 * Find line string in area ar that borders Lanelet ll if ar precedes or follows ll
 * @param ar Lanelet
 * @param ll Area that is preceding or following ll
 * @return LineString3d if it exists
 */
inline Optional<ConstLineString3d> determineCommonLine(const ConstArea& ar, const ConstLanelet& ll);

/**
 * Find Line String in Area ar1 that is common with Area ar2
 * @param ar1 Area
 * @param ar2 Area
 * @return LineString3d in Area ar1 if it exists. The inverted line string is part of ar2.
 */
inline Optional<ConstLineString3d> determineCommonLine(const ConstArea& ar1, const ConstArea& ar2);
}  // namespace geometry
}  // namespace lanelet

#include "impl/Area.h"
