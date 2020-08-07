// this is for emacs file handling -*- mode: c++; c-basic-offset: 2;
// indent-tabs-mode: nil -*-

#pragma once
#include <boost/variant/variant.hpp>
#include <memory>
#include <string>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/utility/HybridMap.h"
#include "lanelet2_core/utility/Optional.h"

namespace lanelet {
namespace internal {
template <typename ValueT>
struct ValueOf {
  using Type = std::decay_t<ValueT>;
};
template <typename ValueT>
struct ValueOf<Optional<ValueT>> {
  using Type = ValueT;
};
}  // namespace internal
   /**
    * @brief An attribute represents one value of a tag of a lanelet primitive.
    *
    * They are internally represented as strings but can be interpreted as numbers.
    *
    * The result of the last as... is cached, therefore repeated calls to the same
    * as.. function are very cheap.
    */
class Attribute {
  template <typename T>
  using ValueOfT = typename internal::ValueOf<T>::Type;

 public:
  using Cache = boost::variant<bool, double, Id, int, Velocity>;

  Attribute() = default;
  Attribute(const std::string& value) : value_{value} {}        // NOLINT
  Attribute(std::string&& value) : value_{std::move(value)} {}  // NOLINT
  Attribute(const char* value) : value_(value) {}               // NOLINT
  Attribute(bool value);                                        // NOLINT
  Attribute(Id value);                                          // NOLINT
  Attribute(int value);                                         // NOLINT
  Attribute(double value);                                      // NOLINT
  Attribute(const Velocity& value);                             // NOLINT

  /**
   * @brief interpret this attribute as bool value
   * @throw ConversionError if the attribute could not be interpreted as bool
   * @return bool value
   */
  Optional<bool> asBool() const;

  /**
   * @brief interpret this attribute as double value
   * @return double if possible to convert the value
   */
  Optional<double> asDouble() const;

  /**
   * @brief interpret this attribute as an id
   * @return id if possible to convert the value
   */
  Optional<Id> asId() const;

  /**
   * @brief interpret this attribute as an int
   * @return int if possible to convert the value
   */
  Optional<int> asInt() const;

  /**
   * @brief interpret this attribute as Velocity
   * @return Velocity if possible to convert the value
   *
   * The function can understand and correctly parse units in the attribute. "2.
   * km/h", "2.mps", etc. To obtain the velocity as a double, divide it by the
   * desired unit:
   *
   * double v = attr.asVelocity() / KMH;
   */
  Optional<Velocity> asVelocity() const;

  /**
   * @brief templated version. Works ony for the as.. above
   */
  template <typename T>
  Optional<ValueOfT<T>> as() const;

  /**
   * @brief gets the value of this attribute
   * @return value
   */
  const std::string& value() const { return value_; }

  /**
   * @brief set the value of this attribute
   * @param value new value
   */
  void setValue(const std::string& value);

 private:
  std::string value_;                     //!< internal value of this parameter
  mutable std::shared_ptr<Cache> cache_;  //!< cache for the last queried value
};

template <>
inline Optional<double> Attribute::as<double>() const {
  return asDouble();
}

template <>
inline Optional<double> Attribute::as<Optional<double>>() const {
  return asDouble();
}

template <>
inline Optional<int> Attribute::as<int>() const {
  return asInt();
}

template <>
inline Optional<int> Attribute::as<Optional<int>>() const {
  return asInt();
}

template <>
inline Optional<bool> Attribute::as<bool>() const {
  return asBool();
}

template <>
inline Optional<bool> Attribute::as<Optional<bool>>() const {
  return asBool();
}

template <>
inline Optional<Id> Attribute::as<Id>() const {
  return asId();
}

template <>
inline Optional<Id> Attribute::as<Optional<Id>>() const {
  return asId();
}

template <>
inline Optional<Velocity> Attribute::as<Velocity>() const {
  return asVelocity();
}

template <>
inline Optional<Velocity> Attribute::as<Optional<Velocity>>() const {
  return asVelocity();
}

template <>
inline Optional<std::string> Attribute::as<std::string>() const {
  return value();
}

template <>
inline Optional<std::string> Attribute::as<Optional<std::string>>() const {
  return value();
}

template <>
inline Optional<const char*> Attribute::as<const char*>() const {
  return value().c_str();
}

template <>
inline Optional<const char*> Attribute::as<Optional<const char*>>() const {
  return value().c_str();
}

inline bool operator==(const Attribute& lhs, const Attribute& rhs) { return lhs.value() == rhs.value(); }
inline bool operator!=(const Attribute& lhs, const Attribute& rhs) { return !(lhs == rhs); }

/**
 * @brief Typical Attributes names within lanelet
 */
enum class AttributeName {
  Type,
  Subtype,
  OneWay,
  ParticipantVehicle,
  ParticipantPedestrian,
  SpeedLimit,
  Location,
  Dynamic
};

using AttributeNamesItem = std::pair<const char*, const AttributeName>;

/**
 * @brief Lists which attribute strings are mapped to which enum value
 *
 * Needs to be in a class because we want external linkage
 */
struct AttributeNamesString {
  static constexpr const char Type[] = "type";
  static constexpr const char Subtype[] = "subtype";
  static constexpr const char OneWay[] = "one_way";
  static constexpr const char ParticipantVehicle[] = "participant:vehicle";
  static constexpr const char ParticipantPedestrian[] = "participant:pedestrian";
  static constexpr const char SpeedLimit[] = "speed_limit";
  static constexpr const char Location[] = "location";
  static constexpr const char Dynamic[] = "dynamic";
  static constexpr const char Color[] = "color";

  // attributes not used in fast lookup
  // on points
  static constexpr const char Ele[] = "ele";

  // on linestrings
  static constexpr const char LaneChange[] = "lane_change";
  static constexpr const char LaneChangeLeft[] = "lane_change:left";
  static constexpr const char LaneChangeRight[] = "lane_change:right";
  static constexpr const char Name[] = "name";
  static constexpr const char Region[] = "region";

  // on lanelets/areas
  static constexpr const char SpeedLimitMandatory[] = "speed_limit_mandatory";
  static constexpr const char Participant[] = "participant";
  static constexpr const char Area[] = "area";
  static constexpr const char Fallback[] = "fallback";
  static constexpr const char Width[] = "width";
  static constexpr const char Height[] = "height";
  static constexpr const char Temporary[] = "temporary";

  // on regulatory elements
  static constexpr const char SignType[] = "sign_type";

  static constexpr AttributeNamesItem Map[] = {{Type, AttributeName::Type},
                                               {Subtype, AttributeName::Subtype},
                                               {OneWay, AttributeName::OneWay},
                                               {ParticipantVehicle, AttributeName::ParticipantVehicle},
                                               {ParticipantPedestrian, AttributeName::ParticipantPedestrian},
                                               {SpeedLimit, AttributeName::SpeedLimit},
                                               {Location, AttributeName::Location},
                                               {Dynamic, AttributeName::Dynamic}};
};

//! parts of tag that have to be combined with either Participants:, OneWay: or SpeedLimit to generate an override.
struct Participants {
  //! Obtain the tag for the participant override
  static std::string tag(const std::string& participant) {
    return AttributeNamesString::Participant + (":" + participant);
  }
  static constexpr const char Vehicle[] = "vehicle";
  static constexpr const char VehicleBus[] = "vehicle:bus";
  static constexpr const char VehicleCar[] = "vehicle:car";
  static constexpr const char VehicleCarElectric[] = "vehicle:car:electric";
  static constexpr const char VehicleCarCombustion[] = "vehicle:car:combustion";
  static constexpr const char VehicleTruck[] = "vehicle:truck";
  static constexpr const char VehicleMotorcycle[] = "vehicle:motorcycle";
  static constexpr const char VehicleTaxi[] = "vehicle:taxi";
  static constexpr const char VehicleEmergency[] = "vehicle:emergency";
  static constexpr const char Pedestrian[] = "pedestrian";
  static constexpr const char Bicycle[] = "bicycle";
  static constexpr const char Train[] = "train";
};
/**
 * @brief Common values for attributes are defined here.
 *
 * This is for convenience when comparing attribute values
 */
struct AttributeValueString {
  // lanelet types
  static constexpr const char Node[] = "node";
  static constexpr const char Way[] = "way";
  static constexpr const char Relation[] = "relation";
  static constexpr const char Lanelet[] = "lanelet";
  static constexpr const char RegulatoryElement[] = "regulatory_element";
  static constexpr const char Multipolygon[] = "multipolygon";

  // line types
  static constexpr const char LineThick[] = "line_thick";
  static constexpr const char LineThin[] = "line_thin";
  static constexpr const char Curbstone[] = "curbstone";
  static constexpr const char GuardRail[] = "guard_rail";
  static constexpr const char RoadBorder[] = "road_border";
  static constexpr const char Wall[] = "wall";
  static constexpr const char Fence[] = "fence";
  static constexpr const char Zebra[] = "zebra_marking";
  static constexpr const char PedestrianMarking[] = "pedestrian_marking";
  static constexpr const char BikeMarking[] = "bike_marking";
  static constexpr const char Keepout[] = "keepout";
  static constexpr const char StopLine[] = "stop_line";
  static constexpr const char Virtual[] = "virtual";
  static constexpr const char Visualization[] = "visualization";
  static constexpr const char ZigZag[] = "zig-zag";
  static constexpr const char LiftGate[] = "lift_gate";
  static constexpr const char JerseyBarrier[] = "jersey_barrier";
  static constexpr const char Gate[] = "gate";
  static constexpr const char Door[] = "door";
  static constexpr const char Trajectory[] = "trajectory";
  static constexpr const char Rail[] = "rail";
  static constexpr const char Bump[] = "bump";

  // line subtypes
  static constexpr const char Solid[] = "solid";
  static constexpr const char Dashed[] = "dashed";
  static constexpr const char DashedSolid[] = "dashed_solid";
  static constexpr const char SolidDashed[] = "solid_dashed";
  static constexpr const char SolidSolid[] = "solid_solid";
  static constexpr const char Straight[] = "straight";
  static constexpr const char Left[] = "left";
  static constexpr const char Right[] = "right";
  static constexpr const char StraightLeft[] = "straight_left";
  static constexpr const char StraightRight[] = "straight_right";
  static constexpr const char LeftRight[] = "left_right";
  static constexpr const char High[] = "high";
  static constexpr const char Low[] = "low";

  // Node types
  static constexpr const char Arrow[] = "arrow";
  static constexpr const char Pole[] = "pole";
  static constexpr const char Post[] = "post";
  static constexpr const char Symbol[] = "symbol";
  static constexpr const char Start[] = "start";
  static constexpr const char End[] = "end";
  static constexpr const char Dot[] = "dot";

  // Color / traffic light types
  static constexpr const char RedYellowGreen[] = "red_yellow_green";
  static constexpr const char RedGreen[] = "red_green";
  static constexpr const char RedYellow[] = "red_yellow";
  static constexpr const char Red[] = "red";
  static constexpr const char Yellow[] = "yellow";
  static constexpr const char White[] = "white";

  // Lanelet types
  static constexpr const char Road[] = "road";
  static constexpr const char Highway[] = "highway";
  static constexpr const char PlayStreet[] = "play_street";
  static constexpr const char EmergencyLane[] = "emergency_lane";
  static constexpr const char BusLane[] = "bus_lane";
  static constexpr const char BicycleLane[] = "bicycle_lane";
  static constexpr const char Walkway[] = "walkway";
  static constexpr const char SharedWalkway[] = "shared_walkway";
  static constexpr const char Crosswalk[] = "crosswalk";
  static constexpr const char Stairs[] = "stairs";

  // Lanelet location tag
  static constexpr const char Nonurban[] = "nonurban";
  static constexpr const char Urban[] = "urban";
  static constexpr const char Private[] = "private";

  // Area types
  static constexpr const char Parking[] = "parking";
  static constexpr const char Freespace[] = "freespace";
  static constexpr const char Vegetation[] = "vegetation";
  static constexpr const char Building[] = "building";
  static constexpr const char TrafficIsland[] = "traffic_island";
  static constexpr const char Exit[] = "exit";

  // Regulatory elements
  static constexpr const char TrafficLight[] = "traffic_light";
  static constexpr const char TrafficSign[] = "traffic_sign";
  static constexpr const char SpeedLimit[] = "speed_limit";
  static constexpr const char RightOfWay[] = "right_of_way";
  static constexpr const char AllWayStop[] = "all_way_stop";
};

inline std::ostream& operator<<(std::ostream& stream, const Attribute& obj) { return stream << obj.value(); }

using AttributeMap = HybridMap<Attribute, decltype(AttributeNamesString::Map)&, AttributeNamesString::Map>;

inline std::ostream& operator<<(std::ostream& stream, const AttributeMap& obj) {
  for (const auto& o : obj) {
    stream << o.first << ": " << o.second << " ";
  }
  return stream;
}
}  // namespace lanelet
