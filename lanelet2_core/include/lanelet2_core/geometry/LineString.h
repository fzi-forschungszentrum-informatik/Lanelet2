#pragma once
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/segment.hpp>
#include "../Forward.h"
#include "../primitives/CompoundLineString.h"
#include "../primitives/LineString.h"
#include "BoundingBox.h"
#include "GeometryHelper.h"
#include "Point.h"

/***********************************************************************
 *                   BOOST GEOMETRY REGISTRATIONS                      *
 ***********************************************************************/

BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::BasicLineString3d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::BasicLineString2d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::LineString3d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::LineString2d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::ConstLineString3d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::ConstLineString2d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::ConstHybridLineString3d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::ConstHybridLineString2d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::CompoundLineString2d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::CompoundLineString3d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::CompoundHybridLineString2d)
BOOST_GEOMETRY_REGISTER_LINESTRING(lanelet::CompoundHybridLineString3d)
BOOST_GEOMETRY_REGISTER_SEGMENT_TEMPLATIZED(lanelet::Segment, first, second);

namespace lanelet {
namespace geometry {

using boost::geometry::intersects;
using boost::geometry::length;
using boost::geometry::overlaps;
using boost::geometry::touches;

template <typename LineStringIterator>
double rangedLength(LineStringIterator start, LineStringIterator end);

/**
 * Calculates the length ratios for the lines in the LineString.
 *
 * The length ratio of a line is the line's length divided by the LineString's
 * length.
 * @param lineString the linestring to do this for.
 * @return A vector of length ratios. Size is lineString.size()-1
 *
 * The function is templated to work on all LineString types, 2d or 3d.
 * Depending on the type, the result will be computed in 2d or 3d.
 */
template <typename LineStringT>
std::vector<double> lengthRatios(const LineStringT& lineString);

/**
 * Calculates the accumulated length ratios for the lines in the LineString.
 *
 * @see lengthRatios, but summed up along the LineString.
 * @param lineString the LineString to do this for.
 * @return A vector of length ratios. Size is lineString.size()-1
 *
 * The last element will aways be (near) 1
 */
template <typename LineStringT>
std::vector<double> accumulatedLengthRatios(const LineStringT& lineString);

/**
 * Calculate the metric signed distance from p to the LineString.
 * The sign is positive if the point is left of the linestring when projecting
 * to the xy-plane.
 *
 * @param lineString the linestring to check for
 * @param p the point to check for
 * @return The metric signed distance in 3d.
 * @todo not sure if this works as expected
 *
 * If the point is before or behind the linestring, this is checked by
 * extrapolating the first or last segment.
 */
template <typename LineString3dT>
double signedDistance(const LineString3dT& lineString, const BasicPoint3d& p);

/**
 * Calculate the metric signed distance from p to the LineString.
 * The sign is positive if the point is left of the linestring when projecting
 * to the xy-plane.
 *
 * @param lineString the linestring to check for
 * @param p point to check for
 * @return the metric signed distance in 2d.
 * @todo not sure if this works as expected
 *
 * If the point is before or behind the linestring, this is checked by
 * extrapolating the first or last segment.
 */
template <typename LineString2dT>
double signedDistance(const LineString2dT& lineString, const BasicPoint2d& p);

/**
 *
 * @brief Transform a point to the coordinates of the linestring
 *
 * This computes the Distance along the LineString and distance to the
 * LineString for the point on the LineString that is closest to the input
 * point.
 */
template <typename LineString2dT>
ArcCoordinates toArcCoordinates(const LineString2dT& lineString, const BasicPoint2d& point);

/**
 * Returns the piecewise linearly interpolated point at the given distance.
 * Negative distances are interpreted backwards from the end.
 * @param lineString the lineString to iterate. Size must be >0.
 * @param dist distance along linestring. If negative, the lineString is
 * iterated in reversed order.
 * @return The interpolated point (a new point if not perfectly matching)
 *
 * This function works in 2d or 3d, depending on the type of the lineString.
 * If the distance is greater length, the end point is returned (or start point
 * if <0).
 */
template <typename LineStringT>
traits::BasicPointT<traits::PointType<LineStringT>> interpolatedPointAtDistance(LineStringT lineString, double dist);

/**
 * @brief returns the cosest point to a position on the linestring
 * @param lineString the lineString to iterate. Size must be >0.
 * @param dist distance along linestring. If negative, the lineString is
 * iterated in reversed order.
 * @return The closest point.
 *
 * This function works in 2d or 3d, depending on the type of the lineString.
 * If the distance is greater length, the end point is returned (or start point
 * if <0).
 */
template <typename LineStringT>
traits::PointType<LineStringT> nearestPointAtDistance(LineStringT lineString, double dist);

//! Get the surrounding axis-aligned bounding box in 3d
template <typename LineString3dT>
IfLS<LineString3dT, BoundingBox3d> boundingBox3d(const LineString3dT& lineString);

//! Get the surrounding axis-aligned bounding box in 2d
template <typename LineString2dT>
IfLS<LineString2dT, BoundingBox2d> boundingBox2d(const LineString2dT& lineString);

/**
 * @brief Projects the given point in 3d to the LineString.
 *
 * If the point is before or behind the lineString, this will be the respective
 * endpoint of the lineString. If not, this will be some interpolated point on
 * the linestring that minimizes the distance between pointToProject and the
 * lineString
 */
template <typename LineString3dT, typename = std::enable_if_t<traits::is3D<LineString3dT>()>>
BasicPoint3d project(const LineString3dT& lineString, const BasicPoint3d& pointToProject);

//! Projects the given point in 2d to the LineString.
template <typename LineString2dT, typename = std::enable_if_t<traits::is2D<LineString2dT>()>>
BasicPoint2d project(const LineString2dT& lineString, const BasicPoint2d& pointToProject);
/**
 * @brief Computes the projected points on the two linestrings for the shortest
 * distance
 *
 * First element of the pair is located on l1, second on l2
 */
template <typename LineString3dT>
IfLS<LineString3dT, std::pair<BasicPoint3d, BasicPoint3d>> projectedPoint3d(const LineString3dT& l1,
                                                                            const LineString3dT& l2);

/**
 * @brief test whether two linestrings intersect in 3d.
 * @param linestring lanelet to check for
 * @param otherLinestring other lanelet to check for
 * @param heightTolerance distance in z below which linestrings are considered
 * as intersecting (in m)
 */
template <typename LineString3dT>
IfLS<LineString3dT, bool> intersects3d(const LineString3dT& linestring, const LineString3dT& otherLinestring,
                                       double heightTolerance = 3.);

/**
 * @brief inverts the two linestrings such that they are parallel
 * @param left the designated left linestring
 * @param right the designated right linestring
 * @return a pair of the left and right linestring (in this order), potentially
 * inverted.
 *
 * Example input:
 * <<<<<<<<LeftLinestring<<<<<<<
 *
 * >>>>>>>>RightLinestring>>>>>>
 *
 * Example output. Left was inverted:
 * >>>>>>>>First>>>>>>>>>>>>>>>>
 *
 * >>>>>>>>Second>>>>>>>>>>>>>>>
 *
 */
template <typename LineString1T, typename LineString2T>
std::pair<LineString1T, LineString2T> align(LineString1T left, LineString2T right);
}  // namespace geometry
}  // namespace lanelet

#include "impl/LineString.h"
