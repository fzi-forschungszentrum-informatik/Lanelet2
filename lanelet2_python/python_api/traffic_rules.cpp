#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/python.hpp>

using namespace boost::python;
using namespace lanelet;
using namespace lanelet::traffic_rules;

SpeedLimitInformation makeSpeedLimit(double speedLimitKph, bool isMandatory) {
  return SpeedLimitInformation{Velocity(speedLimitKph * units::KmH()), isMandatory};
}

double getVelocity(const SpeedLimitInformation& self) { return units::KmHQuantity(self.speedLimit).value(); }

void setVelocity(SpeedLimitInformation& self, double velocityKmh) {
  self.speedLimit = Velocity(velocityKmh * units::KmH());
}

double getVelocityMPS(const SpeedLimitInformation& self) { return units::MPSQuantity(self.speedLimit).value(); }

void setVelocityMPS(SpeedLimitInformation& self, double velocityMps) {
  self.speedLimit = Velocity(velocityMps * units::MPS());
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
      .add_property("speedLimitKmH", getVelocity, setVelocity, "velocity in km/h")
      .add_property("speedLimitMPS", getVelocityMPS, setVelocityMPS, "velocity in m/s")
      .def_readwrite("isMandatory", &SpeedLimitInformation::isMandatory,
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
      .def("location", &TrafficRules::location, return_value_policy<copy_const_reference>(),
           "Location these rules are valid for")
      .def("participant", &TrafficRules::participant, return_value_policy<copy_const_reference>(),
           "Participants the rules are valid for")
      .def(self_ns::str(self_ns::self));

  class_<Locations>("Locations").add_static_property("Germany", asString<Locations::Germany>);

  class_<Participants>("Participants")
      .add_static_property("Vehicle", asString<Participants::Vehicle>)
      .add_static_property("VehicleCar", asString<Participants::VehicleCar>)
      .add_static_property("VehicleCarElectric", asString<Participants::VehicleCarElectric>)
      .add_static_property("VehicleCarCombustion", asString<Participants::VehicleCarCombustion>)
      .add_static_property("VehicleBus", asString<Participants::VehicleBus>)
      .add_static_property("VehicleTruck", asString<Participants::VehicleTruck>)
      .add_static_property("VehicleMotorcycle", asString<Participants::VehicleMotorcycle>)
      .add_static_property("VehicleTaxi", asString<Participants::VehicleTaxi>)
      .add_static_property("VehicleEmergency", asString<Participants::VehicleEmergency>)
      .add_static_property("Bicycle", asString<Participants::Bicycle>)
      .add_static_property("Pedestrian", asString<Participants::Pedestrian>)
      .add_static_property("Train", asString<Participants::Train>);

  def("create", createTrafficRulesWrapper,
      "Create a traffic rules object from location and participant string (see "
      "Locations and Participants class");
}
