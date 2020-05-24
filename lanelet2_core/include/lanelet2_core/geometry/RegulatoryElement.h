#pragma once
#include "lanelet2_core/primitives/BoundingBox.h"
#include "lanelet2_core/primitives/RegulatoryElement.h"

namespace lanelet {
namespace geometry {
/**
 * @brief compute the 2d bounding box around all RuleParameters contained in
 * this RegulatoryElement
 * @see boundingBox3d
 *
 * Note that this does not include the lanelet that refers to this regulatry
 * element, only the the roles in the regulatory element. The bounding box can
 * also be empty if the RegulatoryElement does not contain any data.
 */
BoundingBox2d boundingBox2d(const RegulatoryElement& regElem);

/**
 * @brief compute bounding box in 2d
 *
 * See non-shared-ptr version.
 */
inline BoundingBox2d boundingBox2d(const RegulatoryElementConstPtr& regElem) { return boundingBox2d(*regElem); }

/**
 * @brief compute the 3d bounding box around all RuleParameters contained in
 * this RegulatoryElement
 * @see boundingBox2d
 *
 * Note that this does not include the lanelet that refers to this regulatry
 * element, only the the roles in the regulatory element. The bounding box can
 * also be empty if the regulatory elemnt does not contain any data.
 */
BoundingBox3d boundingBox3d(const RegulatoryElement& regElem);
inline BoundingBox3d boundingBox3d(const RegulatoryElementConstPtr& regElem) { return boundingBox3d(*regElem); }

//! computes the distance between a point and the closest RuleParameter of a
//! RegulatoryElement
double distance2d(const RegulatoryElement& regElem, const BasicPoint2d& p);

//! See non-shared-ptr version
inline double distance2d(const RegulatoryElementConstPtr& regElem1, const BasicPoint2d& p) {
  return distance2d(*regElem1, p);
}
}  // namespace geometry
}  // namespace lanelet
