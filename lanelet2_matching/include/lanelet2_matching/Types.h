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

#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lanelet {
namespace matching {

using Pose2d = Eigen::Transform<double, 2, Eigen::Isometry, Eigen::DontAlign>;  //!< a 2d pose
using PositionCovariance2d = Eigen::Matrix<double, 2, 2, Eigen::DontAlign>;     //!< a covariance of a 2d position
using Hull2d = BasicPolygon2d;                                                  //!< a hull of 2d-points, as
                                                                                //! objects are usually closed rings,
                                                                                //! closing the ring by
                                                                                //! appending the first point
                                                                                //! of the polygon as last
                                                                                //! point again is suggested

struct Object2d {
  Id objectId{InvalId};             //!< Id as convenience for the user, not used by this library
  Pose2d pose{Pose2d::Identity()};  //!< Pose of the object in map coordinates
  Hull2d absoluteHull;  //!< Hull of the object in map coordinates, position is used for matching when hull is empty
};

struct ObjectWithCovariance2d : Object2d {
  PositionCovariance2d positionCovariance{PositionCovariance2d::Zero()};
  double vonMisesKappa{0.};  //!< kappa as defined in https://en.wikipedia.org/wiki/Von_Mises_distribution
};

struct LaneletMatch {
  Lanelet lanelet;
  double distance{0.};  //!< euclidean distance to lanelet
};

struct ConstLaneletMatch {
  ConstLanelet lanelet;
  double distance{0.};  //!< euclidean distance to lanelet
};

struct LaneletMatchProbabilistic : LaneletMatch {
  double mahalanobisDistSq{0.};  //!< squared Mahalanobis distance to closest point on centerline of lanelet
};

struct ConstLaneletMatchProbabilistic : ConstLaneletMatch {
  double mahalanobisDistSq{0.};  //!< squared Mahalanobis distance to closest point on centerline of lanelet
};

}  // namespace matching
}  // namespace lanelet
