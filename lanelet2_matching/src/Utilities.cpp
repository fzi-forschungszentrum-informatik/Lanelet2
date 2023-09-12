/*
 * Copyright (c) 2019
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lanelet2_matching/Utilities.h"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <cmath>

namespace {

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
// Result is always positive; not similar to fmod()
double positiveFloatModulo(double x, double y) {
  double fmod = std::fmod(x, y);
  if (fmod > 0.) {
    return fmod;
  }
  double fmodPositive = fmod + std::abs(y);
  assert(fmodPositive > 0.);
  return fmodPositive;
}

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
double normalizeAngleRadians(double x) { return positiveFloatModulo((x + M_PI), 2.0 * M_PI) - M_PI; }

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
double angleDifference(double targetAngle, double sourceAngle) {
  double angleDiff = targetAngle - sourceAngle;
  return normalizeAngleRadians(angleDiff);
}

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
double yawFromIsometry2d(const Eigen::Isometry2d& pose) {
  Eigen::Rotation2D<double> rot;
  rot.fromRotationMatrix(pose.linear());
  return rot.smallestAngle();
}
}  // namespace

namespace lanelet {
namespace matching {
namespace utils {

double getMahalanobisDistSq(const ConstLanelet& lanelet, const ObjectWithCovariance2d& obj) {
  if (obj.positionCovariance.isZero()) {
    throw MatchingError("Covariance must not be zero");
  }
  if (fabs(obj.positionCovariance.determinant()) < 10e-9) {
    throw MatchingError("Determinant must not be zero");
  }
  auto centerline2d = lanelet::utils::to2D(lanelet.centerline());
  ArcCoordinates closestOnCenter = geometry::toArcCoordinates(centerline2d, obj.pose.translation());
  BasicPoint2d pAt = geometry::interpolatedPointAtDistance(centerline2d, closestOnCenter.length);
  BasicPoint2d pBefore =
      geometry::interpolatedPointAtDistance(centerline2d, std::max(closestOnCenter.length - 0.5, 0.));
  BasicPoint2d pAfter = geometry::interpolatedPointAtDistance(centerline2d, closestOnCenter.length + 0.5);
  BasicPoint2d pDirection = pAfter - pBefore;

  double yawCenter = normalizeAngleRadians(std::atan2(pDirection.y(), pDirection.x()));
  double yawObj = normalizeAngleRadians(yawFromIsometry2d(obj.pose));
  double yawDiff = angleDifference(yawCenter, yawObj);

  // using approximation orientationCovariance = 1./obj.vonMisesKappa
  double mahaDistYawSq = (yawDiff * yawDiff) * (obj.vonMisesKappa * obj.vonMisesKappa);

  BasicPoint2d pDiff = obj.pose.translation() - pAt;
  double mahaDistPosSq = pDiff.transpose() * obj.positionCovariance.inverse() * pDiff;

  return mahaDistYawSq + mahaDistPosSq;
}

}  // namespace utils
}  // namespace matching
}  // namespace lanelet
