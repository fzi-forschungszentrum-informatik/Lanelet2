/*
 * Copyright (c) 2020
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

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_matching/Types.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "lanelet2_python/internal/converter.h"

namespace {
using namespace boost::python;
using namespace lanelet;
using namespace lanelet::matching;

std::string printPoseWrapper(const Pose2d& p) {
  std::ostringstream oss;
  oss << "x: " << p.translation().x() << "\ny:  " << p.translation().y()
      << "\nphi: " << Eigen::Rotation2Dd(p.rotation()).angle();
  return oss.str();
}

boost::shared_ptr<Pose2d> createPose(const double x, const double y, const double phi) {
  Eigen::Rotation2Dd rot(phi);
  auto ptr = boost::make_shared<Pose2d>(rot);
  ptr->translation() = Eigen::Vector2d(x, y);
  return ptr;
}

Hull2d hullFromList(const boost::python::list& l) {
  Hull2d poly{boost::python::stl_input_iterator<BasicPoint2d>(l), boost::python::stl_input_iterator<BasicPoint2d>()};
  return poly;
}

void clearPythonErrors() {
  PyObject *ptype, *pvalue, *ptraceback;  // NOLINT
  PyErr_Fetch(&ptype, &pvalue, &ptraceback);
  // create boost python objects from objects behind pointer, such that python does garbage collection
  if (!!ptype) {  // NOLINT
    object(handle<>(ptype));
  }
  if (!!pvalue) {  // NOLINT
    object(handle<>(pvalue));
  }
  if (!!ptraceback) {  // NOLINT
    object(handle<>(ptraceback));
  }
}

template <typename T>
std::vector<T> removeNonRuleCompliantMatchesImpl(const boost::python::list& matchesList,
                                                 const lanelet::traffic_rules::TrafficRulesPtr& trafficRulesPtr) {
  // convert list to a vector of type T (throws a python exception if not possible)
  std::vector<T> matchesVector{boost::python::stl_input_iterator<T>(matchesList),
                               boost::python::stl_input_iterator<T>()};
  return removeNonRuleCompliantMatches<std::vector<T>>(matchesVector, trafficRulesPtr);
}

object ruleCheckWrapper(const boost::python::list& matches,
                        const lanelet::traffic_rules::TrafficRulesPtr& trafficRulesPtr) {
  try {
    return object(removeNonRuleCompliantMatchesImpl<ConstLaneletMatchProbabilistic>(matches, trafficRulesPtr));
  } catch (const boost::python::error_already_set&) {
    clearPythonErrors();
  }
  try {
    return object(removeNonRuleCompliantMatchesImpl<ConstLaneletMatch>(matches, trafficRulesPtr));
  } catch (const boost::python::error_already_set&) {
    clearPythonErrors();
  }
  throw std::runtime_error("Matches must be a list of ConstLaneletMatch(es) or ConstLaneletMatch(es)Probabilistic");
}

boost::shared_ptr<PositionCovariance2d> covFromVarXVarYCovXY(const double varX, const double varY, const double covXY) {
  PositionCovariance2d cov;
  cov << varX, covXY, covXY, varY;
  return boost::make_shared<PositionCovariance2d>(cov);
}

std::vector<ConstLaneletMatch> (*funcWrapperDeterministic)(const LaneletMap&, const Object2d&,
                                                           const double) = &getDeterministicMatches;
std::vector<ConstLaneletMatchProbabilistic> (*funcWrapperProbabilistic)(const LaneletMap&,
                                                                        const ObjectWithCovariance2d&,
                                                                        const double) = &getProbabilisticMatches;
}  // namespace
using ::converters::VectorToListConverter;

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  auto core = import("lanelet2.core");

  class_<Pose2d>("Pose2d", "2D Isometric Transformation", no_init)
      .def("__init__",
           make_constructor(&createPose, default_call_policies(), (arg("x") = 0., arg("y") = 0, arg("yaw") = 0.)))
      .def("__str__", &printPoseWrapper);

  class_<Object2d>("Object2d", "Object with pose, hull and ID", no_init)
      .def("__init__", make_constructor(
                           +[](lanelet::Id objectId, const Pose2d& pose, const boost::python::list& absoluteHull) {
                             return boost::make_shared<Object2d>(Object2d{objectId, pose, hullFromList(absoluteHull)});
                           },
                           default_call_policies(),
                           (arg("objectId") = lanelet::InvalId, arg("pose") = Pose2d::Identity(),
                            arg("absoluteHull") = boost::python::list())))
      .add_property(
          "absoluteHull", +[](Object2d& self) { return self.absoluteHull; }, &hullFromList,
          "hull in absolute coordinates (not relative to the object's pose)")
      .def_readwrite("pose", &Object2d::pose)
      .def_readwrite("objectId", &Object2d::objectId);

  class_<PositionCovariance2d>("PositionCovariance2d", init<>())
      .def("__init__", make_constructor(&covFromVarXVarYCovXY, default_call_policies(),
                                        (arg("varX") = 0., arg("varY") = 0., arg("covXY") = 0.)))
      .def(self_ns::str(self_ns::self));

  class_<ObjectWithCovariance2d, bases<Object2d>>("ObjectWithCovariance2d", "Object with pose, covariance, hull and ID",
                                                  no_init)
      .def("__init__", make_constructor(
                           +[](lanelet::Id objectId, const Pose2d& pose, const boost::python::list& absoluteHull,
                               const PositionCovariance2d& positionCovariance, double vonMisesKappa) {
                             ObjectWithCovariance2d obj;
                             obj.objectId = objectId;
                             obj.pose = pose;
                             obj.absoluteHull = hullFromList(absoluteHull);
                             obj.positionCovariance = positionCovariance;
                             obj.vonMisesKappa = vonMisesKappa;
                             return boost::make_shared<ObjectWithCovariance2d>(obj);
                             // initializer list construction of derived struct requires cpp17
                             // return boost::make_shared<ObjectWithCovariance2d>(ObjectWithCovariance2d{
                             //     objectId, pose, hullFromList(absoluteHull), positionCovariance, vonMisesKappa});
                           },
                           default_call_policies(),
                           (arg("objectId") = lanelet::InvalId, arg("pose") = Pose2d::Identity(),
                            arg("absoluteHull") = boost::python::list(),
                            arg("positionCovariance") = PositionCovariance2d(), arg("vonMisesKappa") = 0.)))
      .def_readwrite("vonMisesKappa", &ObjectWithCovariance2d::vonMisesKappa)
      .def_readwrite("positionCovariance", &ObjectWithCovariance2d::positionCovariance);

  class_<ConstLaneletMatch>("ConstLaneletMatch", "Results from matching objects to lanelets", no_init)
      .def_readwrite("lanelet", &ConstLaneletMatch::lanelet)
      .def_readwrite("distance", &ConstLaneletMatch::distance);

  class_<ConstLaneletMatchProbabilistic, bases<ConstLaneletMatch>>("ConstLaneletMatchProbabilistic",
                                                                   "Results from matching objects to lanelets", no_init)
      .def_readwrite("mahalanobisDistSq", &ConstLaneletMatchProbabilistic::mahalanobisDistSq);

  VectorToListConverter<std::vector<ConstLaneletMatch>>();
  VectorToListConverter<std::vector<ConstLaneletMatchProbabilistic>>();

  def("getDeterministicMatches", funcWrapperDeterministic);
  def("getProbabilisticMatches", funcWrapperProbabilistic);
  def("removeNonRuleCompliantMatches", ruleCheckWrapper);
}
