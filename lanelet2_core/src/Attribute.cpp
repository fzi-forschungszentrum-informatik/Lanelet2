#include "lanelet2_core/Attribute.h"

#include <boost/lexical_cast.hpp>
#include <boost/variant/get.hpp>
#include <regex>

#include "lanelet2_core/Exceptions.h"
#include "lanelet2_core/utility/Units.h"

namespace lanelet {
namespace {
Optional<Velocity> getUnit(const std::string& value, size_t pos) {
  using namespace units::literals;
  if (pos >= value.size()) {
    return 1._kmh;
  }
  auto unit = value.substr(pos);
  const auto flags = std::regex::ECMAScript;
  // try to match unit
  if (std::regex_search(unit, std::regex("\\s*(m/s)|(mps)", flags))) {
    return 1_mps;
  }
  if (std::regex_search(unit, std::regex("\\s*(km/h)|(kmh)", flags))) {
    return 1_kmh;
  }
  if (std::regex_search(unit, std::regex("\\s*(m/h)|(mph)", flags))) {
    return 1_mph;
  }
  return {};
}

template <typename T>
T store(std::shared_ptr<Attribute::Cache>& cache, T&& value) {
  auto newCache = std::make_shared<Attribute::Cache>(value);
  std::atomic_store_explicit(&cache, newCache, std::memory_order_release);
  return value;
}

template <typename T>
T* load(const std::shared_ptr<Attribute::Cache>& cache) {
  auto c = std::atomic_load_explicit(&cache, std::memory_order_acquire);
  if (!c) {
    return nullptr;
  }
  return boost::get<T>(&*cache);
}
}  // namespace

Attribute::Attribute(Id value) : value_(std::to_string(value)) { store(cache_, value); }

Attribute::Attribute(bool value) : value_(std::to_string(int(value))) { store(cache_, value); }

Attribute::Attribute(int value) : value_(std::to_string(value)) { store(cache_, value); }

Attribute::Attribute(double value) : value_(std::to_string(value)) { store(cache_, value); }

Attribute::Attribute(const Velocity& value) : value_{std::to_string(units::KmHQuantity(value).value())} {
  store(cache_, value);
}

Optional<bool> Attribute::asBool() const {
  // try load from cache
  auto* val = load<bool>(cache_);
  if (val != nullptr) {
    return *val;
  }
  // need to compute value
  try {
    return boost::lexical_cast<bool>(value());
  } catch (boost::bad_lexical_cast&) {
    if (value() == "true" || value() == "yes") {
      return store(cache_, true);
    }
    if (value() == "false" || value() == "no") {
      return store(cache_, false);
    }
    return {};
  }
}

Optional<double> Attribute::asDouble() const {
  // try load from cache
  auto* val = load<double>(cache_);
  if (val != nullptr) {
    return *val;
  }
  // need to compute value
  try {
    return store(cache_, boost::lexical_cast<double>(value()));
  } catch (boost::bad_lexical_cast&) {
    return {};
  }
}

Optional<Id> Attribute::asId() const {
  // try load from cache
  auto* val = load<Id>(cache_);
  if (val != nullptr) {
    return *val;
  }
  // need to compute value
  try {
    return store(cache_, boost::lexical_cast<Id>(value()));
  } catch (boost::bad_lexical_cast&) {
    return {};
  }
}

Optional<int> Attribute::asInt() const {
  // try load from cache
  auto* val = load<int>(cache_);
  if (val != nullptr) {
    return *val;
  }
  // need to compute value
  try {
    return store(cache_, boost::lexical_cast<int>(value()));
  } catch (boost::bad_lexical_cast&) {
    return {};
  }
}

Optional<Velocity> Attribute::asVelocity() const {
  // try load from cache
  auto* val = load<Velocity>(cache_);
  if (val != nullptr) {
    return *val;
  }
  // need to compute value
  auto asD = asDouble();
  if (asD) {
    // we assume kmh by default
    return store(cache_, Velocity(*asD * units::KmH()));
  }
  // try to extract a unit (assuming something like "2 mps")
  try {
    size_t idx = 0;
    const auto velocity = std::stod(value(), &idx);
    auto unit = getUnit(value(), idx);
    if (unit) {
      return store(cache_, velocity * *unit);
    }
  } catch (std::exception&) {
  }
  // ok, give up
  return {};
}

void Attribute::setValue(const std::string& value) {
  std::atomic_store_explicit(&cache_, std::shared_ptr<Cache>(), std::memory_order_release);
  this->value_ = value;
}

#if __cplusplus < 201703L
// see https://regexr.com/3knio
// lanelet types
constexpr const char AttributeValueString::Node[];
constexpr const char AttributeValueString::Way[];
constexpr const char AttributeValueString::Relation[];
constexpr const char AttributeValueString::Lanelet[];
constexpr const char AttributeValueString::RegulatoryElement[];
constexpr const char AttributeValueString::Multipolygon[];

// line types
constexpr const char AttributeValueString::LineThick[];
constexpr const char AttributeValueString::LineThin[];
constexpr const char AttributeValueString::Curbstone[];
constexpr const char AttributeValueString::GuardRail[];
constexpr const char AttributeValueString::RoadBorder[];
constexpr const char AttributeValueString::Wall[];
constexpr const char AttributeValueString::Fence[];
constexpr const char AttributeValueString::Zebra[];
constexpr const char AttributeValueString::PedestrianMarking[];
constexpr const char AttributeValueString::BikeMarking[];
constexpr const char AttributeValueString::Keepout[];
constexpr const char AttributeValueString::StopLine[];
constexpr const char AttributeValueString::Virtual[];
constexpr const char AttributeValueString::Visualization[];
constexpr const char AttributeValueString::ZigZag[];
constexpr const char AttributeValueString::LiftGate[];
constexpr const char AttributeValueString::JerseyBarrier[];
constexpr const char AttributeValueString::Gate[];
constexpr const char AttributeValueString::Door[];
constexpr const char AttributeValueString::Trajectory[];
constexpr const char AttributeValueString::Rail[];
constexpr const char AttributeValueString::Bump[];

// line subtypes
constexpr const char AttributeValueString::Solid[];
constexpr const char AttributeValueString::Dashed[];
constexpr const char AttributeValueString::DashedSolid[];
constexpr const char AttributeValueString::SolidDashed[];
constexpr const char AttributeValueString::SolidSolid[];
constexpr const char AttributeValueString::Straight[];
constexpr const char AttributeValueString::Left[];
constexpr const char AttributeValueString::Right[];
constexpr const char AttributeValueString::StraightLeft[];
constexpr const char AttributeValueString::StraightRight[];
constexpr const char AttributeValueString::LeftRight[];
constexpr const char AttributeValueString::High[];
constexpr const char AttributeValueString::Low[];

// Node types
constexpr const char AttributeValueString::Arrow[];
constexpr const char AttributeValueString::Pole[];
constexpr const char AttributeValueString::Post[];
constexpr const char AttributeValueString::Symbol[];
constexpr const char AttributeValueString::Start[];
constexpr const char AttributeValueString::End[];
constexpr const char AttributeValueString::Dot[];

// Color / traffic light types
constexpr const char AttributeValueString::RedYellowGreen[];
constexpr const char AttributeValueString::RedGreen[];
constexpr const char AttributeValueString::RedYellow[];
constexpr const char AttributeValueString::Red[];
constexpr const char AttributeValueString::Yellow[];
constexpr const char AttributeValueString::White[];

// Lanelet types
constexpr const char AttributeValueString::Road[];
constexpr const char AttributeValueString::Highway[];
constexpr const char AttributeValueString::PlayStreet[];
constexpr const char AttributeValueString::BusLane[];
constexpr const char AttributeValueString::EmergencyLane[];
constexpr const char AttributeValueString::BicycleLane[];
constexpr const char AttributeValueString::Walkway[];
constexpr const char AttributeValueString::SharedWalkway[];
constexpr const char AttributeValueString::Crosswalk[];
constexpr const char AttributeValueString::Stairs[];

// Lanelet location tag
constexpr const char AttributeValueString::Nonurban[];
constexpr const char AttributeValueString::Urban[];
constexpr const char AttributeValueString::Private[];

// Area types
constexpr const char AttributeValueString::Parking[];
constexpr const char AttributeValueString::Freespace[];
constexpr const char AttributeValueString::Vegetation[];
constexpr const char AttributeValueString::Building[];
constexpr const char AttributeValueString::TrafficIsland[];
constexpr const char AttributeValueString::Exit[];

// Regulatory elements
constexpr const char AttributeValueString::TrafficLight[];
constexpr const char AttributeValueString::TrafficSign[];
constexpr const char AttributeValueString::SpeedLimit[];
constexpr const char AttributeValueString::RightOfWay[];
constexpr const char AttributeValueString::AllWayStop[];

// other
constexpr const char AttributeNamesString::Type[];
constexpr const char AttributeNamesString::Subtype[];
constexpr const char AttributeNamesString::OneWay[];
constexpr const char AttributeNamesString::ParticipantVehicle[];
constexpr const char AttributeNamesString::ParticipantPedestrian[];
constexpr const char AttributeNamesString::SpeedLimit[];
constexpr const char AttributeNamesString::Location[];
constexpr const char AttributeNamesString::Dynamic[];
constexpr const char AttributeNamesString::Color[];

// attributes not used in fast lookup
// on points
constexpr const char AttributeNamesString::Ele[];

// on linestrings
constexpr const char AttributeNamesString::LaneChange[];
constexpr const char AttributeNamesString::LaneChangeLeft[];
constexpr const char AttributeNamesString::LaneChangeRight[];
constexpr const char AttributeNamesString::Name[];
constexpr const char AttributeNamesString::Region[];

// on lanelets/areas
constexpr const char AttributeNamesString::SpeedLimitMandatory[];
constexpr const char AttributeNamesString::Area[];
constexpr const char AttributeNamesString::Participant[];
constexpr const char AttributeNamesString::Fallback[];
constexpr const char AttributeNamesString::Width[];
constexpr const char AttributeNamesString::Height[];
constexpr const char AttributeNamesString::Temporary[];

// on regulatory elements
constexpr const char AttributeNamesString::SignType[];

// participants
constexpr const char Participants::Vehicle[];
constexpr const char Participants::VehicleBus[];
constexpr const char Participants::VehicleCar[];
constexpr const char Participants::VehicleCarElectric[];
constexpr const char Participants::VehicleCarCombustion[];
constexpr const char Participants::VehicleTruck[];
constexpr const char Participants::VehicleMotorcycle[];
constexpr const char Participants::VehicleTaxi[];
constexpr const char Participants::VehicleEmergency[];
constexpr const char Participants::Pedestrian[];
constexpr const char Participants::Bicycle[];
constexpr const char Participants::Train[];

constexpr AttributeNamesItem AttributeNamesString::Map[];
#endif
}  // namespace lanelet
