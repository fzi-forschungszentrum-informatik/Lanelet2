#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <boost/python.hpp>

using namespace boost::python;
using namespace lanelet;

SpeedLimitInformation makeSpeedLimit(double speedLimitKph, bool isMandatory) {
  return SpeedLimitInformation{Velocity(speedLimitKph * units::KmH()), isMandatory};
}

double getVelocity(const SpeedLimitInformation& self) { return units::KmHQuantity(self.speedLimit).value(); }

void setVelocity(SpeedLimitInformation& self, double velocityKmh) {
  self.speedLimit = Velocity(velocityKmh * units::KmH());
}

template <typename T>
bool canPassWrapper(const TrafficRules& self, const T& llt) {
  return self.canPass(llt);
}
template <typename T1, typename T2>
bool canPassFromToWrapper(const TrafficRules& self, const T1& from, const T2& to) {
  return self.canPass(from, to);
}

template <typename T>
SpeedLimitInformation speedLimitWrapper(const TrafficRules& self, const T& llt) {
  return self.speedLimit(llt);
}
bool isOneWayWrapper(const TrafficRules& self, const ConstLanelet& llt) { return self.isOneWay(llt); }
bool hasDynamicRulesWrapper(const TrafficRules& self, const ConstLanelet& llt) { return self.hasDynamicRules(llt); }
bool hasSameTrafficRulesWrapper(const TrafficRules& self, const ConstLanelet& llt, const ConstLanelet& other) {
  return self.hasSameTrafficRules(llt, other);
}
bool canOvertakeLeftWrapper(const TrafficRules& self, const ConstLanelet& llt) { return self.canOvertakeLeft(llt); }
bool canOvertakeRightWrapper(const TrafficRules& self, const ConstLanelet& llt) { return self.canOvertakeRight(llt); }
bool rightHandTrafficWrapper(const TrafficRules& self) { return self.rightHandTraffic(); }

TrafficRulesPtr createTrafficRulesWrapper(const std::string& location, const std::string& participant) {
  return TrafficRulesFactory::create(location, participant);
}

template <const char Val[]>
std::string asString() {
  return Val;
}

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  auto core = import("lanelet2.core");

  class_<SpeedLimitInformation>("SpeedLimitInformation", "Current speed limit as returned by a traffic rule object")
      .def("__init__", makeSpeedLimit,
           "Initialize from speed limit [m/s] and bool if speedlimit is "
           "mandatory")
      .add_property("speedLimit", getVelocity, setVelocity, "velocity in km/h")
      .add_property("isMandatory", &SpeedLimitInformation::isMandatory,
                    "True if speedlimit is not just a recommendation")
      .def(self_ns::str(self_ns::self));

  class_<TrafficRules, boost::noncopyable, std::shared_ptr<TrafficRules>>("TrafficRules", no_init)
      .def("canPass", canPassWrapper<ConstLanelet>, "Returns whether it is allowed to pass/drive on this lanelet")
      .def("canPass", canPassWrapper<ConstArea>, "Returns whether it is allowed to pass/drive on this area")
      .def("canPass", canPassFromToWrapper<ConstLanelet, ConstLanelet>,
           "Returns whether it is allowed to drive from first to second lanelet")
      .def("canPass", canPassFromToWrapper<ConstLanelet, ConstArea>)
      .def("canPass", canPassFromToWrapper<ConstArea, ConstArea>)
      .def("canPass", canPassFromToWrapper<ConstArea, ConstLanelet>)
      .def("canChangeLane", &TrafficRules::canChangeLane,
           "determines if a lane change can be made between two lanelets")
      .def("speedLimit", speedLimitWrapper<ConstLanelet>, "get speed limit of this lanelet")
      .def("speedLimit", speedLimitWrapper<ConstArea>, "get speed limit of this lanelet")
      .def("isOneWay", isOneWayWrapper, "returns whether a lanelet can be driven in one direction only")
      .def("hasDynamicRules", hasDynamicRulesWrapper,
           "returns whether dynamic traffic rules apply to this lanelet that "
           "can not be understood by this traffic rules object")
      .def("hasSameTrafficRules", hasSameTrafficRulesWrapper,
           "checks whether traffic restrictions change between two lanelets.")
      .def("canOvertakeLeft", canOvertakeLeftWrapper, "True if overtaking to the left is possible on this lanelet")
      .def("canOvertakeRight", canOvertakeRightWrapper, "True if overtaking right is possible with this lanelet")
      .def("rightHandTraffic", rightHandTrafficWrapper,
           "true if traffic is right handed in the location where this traffic "
           "rules object is located")
      .def("location", &TrafficRules::location, "Location these rules are valid for")
      .def("participant", &TrafficRules::participant, "Participants the rules are valid for")
      .def(self_ns::str(self_ns::self));

  class_<Locations>("Locations").add_static_property("Germany", asString<Locations::Germany>);

  class_<Participants>("Participants")
      .add_static_property("Vehicle", asString<Participants::Vehicle>)
      .add_static_property("Car", asString<Participants::Car>)
      .add_static_property("Bus", asString<Participants::Bus>)
      .add_static_property("Truck", asString<Participants::Truck>)
      .add_static_property("Bicycle", asString<Participants::Bicycle>)
      .add_static_property("Pedestrian", asString<Participants::Pedestrian>);

  def("create", createTrafficRulesWrapper,
      "Create a traffic rules object from location and participant string (see "
      "Locations and Participants class");
}
