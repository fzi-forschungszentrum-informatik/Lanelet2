#pragma once

#include <string>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/primitives/LineStringOrPolygon.h"
#include "lanelet2_core/primitives/RegulatoryElement.h"

namespace lanelet {

/**
 * @brief Represents a traffic light restriction on the lanelet
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */
class TrafficLight : public RegulatoryElement {
 public:
  using Ptr = std::shared_ptr<TrafficLight>;
  static constexpr char RuleName[] = "traffic_light";
  //! Directly construct a stop line from its required rule parameters.
  //! Might modify the input data in oder to get correct tags.
  static Ptr make(Id id, const AttributeMap& attributes, const LineStringsOrPolygons3d& trafficLights,
                  const Optional<LineString3d>& stopLine = {}) {
    return Ptr{new TrafficLight(id, attributes, trafficLights, stopLine)};
  }

  /**
   * @brief get the stop line for the traffic light
   * @return the stop line as LineString
   */
  Optional<ConstLineString3d> stopLine() const;
  Optional<LineString3d> stopLine();

  /**
   * @brief get the relevant traffic lights
   * @return the traffic lights
   *
   * There might be multiple traffic lights but they are required to show the
   * same signal.
   */
  ConstLineStringsOrPolygons3d trafficLights() const;
  LineStringsOrPolygons3d trafficLights();

  /**
   * @brief add a new traffic light
   * @param primitive the traffic light to add
   *
   * Traffic lights are represented as linestrings that start at the left edge
   * of a traffic light and end at the right edge.
   */
  void addTrafficLight(const LineStringOrPolygon3d& primitive);

  /**
   * @brief remove a traffic light
   * @param primitive the primitive
   * @return true if the traffic light existed and was removed
   */
  bool removeTrafficLight(const LineStringOrPolygon3d& primitive);

  /**
   * @brief set a new stop line, overwrite the old one
   * @param stopLine new stop line
   */
  void setStopLine(const LineString3d& stopLine);

  //! Deletes the stop line
  void removeStopLine();

 protected:
  friend class RegisterRegulatoryElement<TrafficLight>;
  TrafficLight(Id id, const AttributeMap& attributes, const LineStringsOrPolygons3d& trafficLights,
               const Optional<LineString3d>& stopLine);
  explicit TrafficLight(const RegulatoryElementDataPtr& data);
};

//! Enum to distinguish maneuver types
enum class ManeuverType {
  Yield,       //!> Lanelet has right of way
  RightOfWay,  //!< Lanelet has to yield
  Unknown      //!< Lanelet ist not part of relation
};

//! @brief Defines right of way restrictions
//! @ingroup RegulatoryElementPrimitives
//! @ingroup Primitives
class RightOfWay : public RegulatoryElement {
 public:
  using Ptr = std::shared_ptr<RightOfWay>;
  static constexpr char RuleName[] = "right_of_way";

  /**
   * @brief Create a valid Right of Way object
   * @param id id for this rule
   * @param attributes for this rule. Might be extended if necessary
   * @param rightOfWay the lanelets that have right of way
   * @param yield the lanelets that have to yield
   * @param stopLine line where to stop. If there is none, stop at the end of
   * the lanelet.
   */
  static Ptr make(Id id, const AttributeMap& attributes, const Lanelets& rightOfWay, const Lanelets& yield,
                  const Optional<LineString3d>& stopLine = {}) {
    return Ptr{new RightOfWay(id, attributes, rightOfWay, yield, stopLine)};
  }

  //! returns whether a lanelet has to yield or has right of way
  ManeuverType getManeuver(const ConstLanelet& lanelet) const;

  //! get the lanelets have right of way
  ConstLanelets rightOfWayLanelets() const;
  Lanelets rightOfWayLanelets();

  //! get the lanelets that have to yield
  ConstLanelets yieldLanelets() const;
  Lanelets yieldLanelets();

  //! get the stop line for the yield lanelets, if present
  Optional<ConstLineString3d> stopLine() const;
  Optional<LineString3d> stopLine();

  //! Overwrites the stop line
  void setStopLine(const LineString3d& stopLine);

  //! Adds a lanelet for RightOfWay
  void addRightOfWayLanelet(const Lanelet& lanelet);

  //! Add yielding lanelet
  void addYieldLanelet(const Lanelet& lanelet);

  //! Removes a right of way lanelet and returns true on success
  bool removeRightOfWayLanelet(const Lanelet& lanelet);

  //! Removes a yielding lanelet and returns true no success
  bool removeYieldLanelet(const Lanelet& lanelet);

  //! Removes the stop line
  void removeStopLine();

 protected:
  friend class RegisterRegulatoryElement<RightOfWay>;
  RightOfWay(Id id, const AttributeMap& attributes, const Lanelets& rightOfWay, const Lanelets& yield,
             const Optional<LineString3d>& stopLine = {});
  explicit RightOfWay(const RegulatoryElementDataPtr& data);
};

struct LaneletWithStopLine {
  Lanelet lanelet;
  Optional<LineString3d> stopLine;
};
struct ConstLaneletWithStopLine {
  ConstLanelet lanelet;
  Optional<ConstLineString3d> stopLine;
};
using LaneletsWithStopLines = std::vector<LaneletWithStopLine>;

//! @brief Defines an all way stop. These are a special form of right of way, where all lanelets have to yield,
//! depending on the order of arrival and the route through the intersection.
//! @ingroup RegulatoryElementPrimitives
//! @ingroup Primitives
//!
//! The distance to the intersection is represented either by the distance to the stop line, if present, otherwise the
//! distance to the end of the lanelet.
class AllWayStop : public RegulatoryElement {
 public:
  using Ptr = std::shared_ptr<AllWayStop>;
  static constexpr char RuleName[] = "all_way_stop";

  /**
   * @brief Create a valid all way stop object
   * @param id id for this rule
   * @param attributes for this rule. Might be extended if necessary
   * @param lltsWithStop lanelets with stop line. Either all lanelets have a stop line or none.
   * @param signs traffic signs that constitute this rule
   */
  static Ptr make(Id id, const AttributeMap& attributes, const LaneletsWithStopLines& lltsWithStop,
                  const LineStringsOrPolygons3d& signs = {}) {
    return Ptr{new AllWayStop(id, attributes, lltsWithStop, signs)};
  }

  //! get the lanelets that potentially have to yield
  ConstLanelets lanelets() const;
  Lanelets lanelets();

  //! Adds a new lanelet with stop line. This will throw if the other lanelets did not have a stop line and vice versa
  //! @throws InvalidInputError if stop line is inconsistent
  void addLanelet(const LaneletWithStopLine& lltWithStop);

  //! Removes a lanelet and the associated stop line, if there is one
  bool removeLanelet(const Lanelet& llt);

  //! get the stop lines
  ConstLineStrings3d stopLines() const;
  LineStrings3d stopLines();

  //! gets the stop line for a lanelet, if there is one
  Optional<ConstLineString3d> getStopLine(const ConstLanelet& llt) const;
  Optional<LineString3d> getStopLine(const ConstLanelet& llt);

  //! get list of traffic signs that constitute this AllWayStop if existing
  ConstLineStringsOrPolygons3d trafficSigns() const;
  LineStringsOrPolygons3d trafficSigns();

  //! Adds another traffic sign
  /** Traffic signs are represented as linestrings that start at the left edge
   * and end at the right edge of a traffic sign.
   */
  void addTrafficSign(const LineStringOrPolygon3d& sign);

  //! removes a traffic sign and returns true on success
  bool removeTrafficSign(const LineStringOrPolygon3d& sign);

 protected:
  friend class RegisterRegulatoryElement<AllWayStop>;
  AllWayStop(Id id, const AttributeMap& attributes, const LaneletsWithStopLines& lltsWithStop,
             const LineStringsOrPolygons3d& signs);
  explicit AllWayStop(const RegulatoryElementDataPtr& data);
};

//! Used as input argument to create TrafficSign regElem
struct TrafficSignsWithType {
  LineStringsOrPolygons3d trafficSigns;  //!< Lists relevant traffic signs
  std::string type{""};                  //! Lists their type. If empty, it is assumed that this
                                         //! is found in the attributes of trafficSigns.
                                         //! the format is <country-code><ID>, eg de205.
};

//! @brief Expresses a generic traffic sign rule
//! @ingroup RegulatoryElementPrimitives
//! @ingroup Primitives
class TrafficSign : public RegulatoryElement {
 public:
  using Ptr = std::shared_ptr<TrafficSign>;
  static constexpr char RuleName[] = "traffic_sign";

  /**
   * @brief Create a valid TrafficSign object
   *
   * A traffic sign is usually composed of a set of traffic signs of the same type that mark the beginning of the rule.
   * It also contains multiple traffic signs (potentially of different types) that mark the end of the rule.
   * E.g. a 50kph section would contain all 50kph signs of this section as trafficSigns. All signs that stand at the end
   * of this section (e.g. a 70kph sign and an end of 50kph sign) would be cancellingTrafficSigns.
   * @param id id of traffic sign rule
   * @param attributes attributes for it (might be extended if necessary)
   * @param trafficSigns list of the traffic signs defining the rule
   * @param cancellingTrafficSigns list of traffic signs where the rule is
   * cancelled. Can be empty.
   * @param refLines lines from where the rule becomes valid. Can be empty.
   * @param cancelLines lines after which a rule becomes invalid. Can be empty.
   */
  static Ptr make(Id id, const AttributeMap& attributes, const TrafficSignsWithType& trafficSigns,
                  const TrafficSignsWithType& cancellingTrafficSigns = {}, const LineStrings3d& refLines = {},
                  const LineStrings3d& cancelLines = {}) {
    return Ptr(new TrafficSign(id, attributes, trafficSigns, cancellingTrafficSigns, refLines, cancelLines));
  }

  //! returns the traffic signs
  /** There might be multiple but they are all required to show the same symbol.
   */
  ConstLineStringsOrPolygons3d trafficSigns() const;
  LineStringsOrPolygons3d trafficSigns();

  //! get the id/number of the sign(s)
  /** The id is in the format [country-code][ID], e.g. de205.
   * The result can be dependant on country
   */
  std::string type() const;

  //! gets the line(s) from which a sign becomes valid.
  /**
   * There might or might not be such a line. If there is none, the sign is
   * valid for the whole lanelet
   */
  ConstLineStrings3d refLines() const;
  LineStrings3d refLines();

  //! get list of cancellingTrafficSigns, if existing
  ConstLineStringsOrPolygons3d cancellingTrafficSigns() const;
  LineStringsOrPolygons3d cancellingTrafficSigns();

  //! Types of the cancelling traffic signs if they exist
  std::vector<std::string> cancelTypes() const;

  //! gets the line(s) from which a sign becomes invalid.
  ConstLineStrings3d cancelLines() const;
  LineStrings3d cancelLines();

  //! Adds another traffic sign
  /** Traffic signs are represented as linestrings that start at the left edge
   * and end at the right edge of a traffic sign.
   */
  void addTrafficSign(const LineStringOrPolygon3d& sign);

  //! remove a traffic sign and returns true on success
  bool removeTrafficSign(const LineStringOrPolygon3d& sign);

  //! Add new cancelling traffic sign
  void addCancellingTrafficSign(const TrafficSignsWithType& signs);

  //! remove a cancelling traffic sign, returns true on success
  bool removeCancellingTrafficSign(const LineStringOrPolygon3d& sign);

  //! Add a new reference line
  void addRefLine(const LineString3d& line);

  //! Remove a reference line. Returns true on success.
  bool removeRefLine(const LineString3d& line);

  //! Add a new line from where the sign becomes inactive
  void addCancellingRefLine(const LineString3d& line);

  //! Remove a cancelling line. Returns true on success.
  bool removeCancellingRefLine(const LineString3d& line);

 protected:
  TrafficSign(Id id, const AttributeMap& attributes, const TrafficSignsWithType& trafficSigns,
              const TrafficSignsWithType& cancellingTrafficSigns = {}, const LineStrings3d& refLines = {},
              const LineStrings3d& cancelLines = {});

  friend class RegisterRegulatoryElement<TrafficSign>;
  explicit TrafficSign(const RegulatoryElementDataPtr& data);
};

/**
 * @brief Represents a speed limit that affects a lanelet
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 *
 * A speed limit is defined by one ore more traffic signs and cancelled by one
 * or more traffic signs. All lanelets affected by this refer to this traffic
 * sign.
 *
 * As an alternative, the type can also be specified using the sign_type tag of
 * the regulatory element. However this is not recommended, because will make it
 * hard to track where the speed limit originates.
 */
class SpeedLimit : public TrafficSign {
 public:
  using Ptr = std::shared_ptr<SpeedLimit>;
  static constexpr char RuleName[] = "speed_limit";

  //! Create a speed limit regulatory element. Similar to a traffic sign.
  static Ptr make(Id id, const AttributeMap& attributes, const TrafficSignsWithType& trafficSigns,
                  const TrafficSignsWithType& cancellingTrafficSigns = {}, const LineStrings3d& refLines = {},
                  const LineStrings3d& cancelLines = {}) {
    return Ptr(new SpeedLimit(id, attributes, trafficSigns, cancellingTrafficSigns, refLines, cancelLines));
  }

  //! Create a speed limit regulatory element only from a type or speed limit without actual sign
  static Ptr make(Id id, AttributeMap attributes, const std::string& signType, const LineStrings3d& refLines = {},
                  const LineStrings3d& cancelLines = {}) {
    attributes.insert(std::make_pair(AttributeNamesString::SignType, signType));
    return Ptr(new SpeedLimit(id, attributes, {}, {}, refLines, cancelLines));
  }

 protected:
  friend class RegisterRegulatoryElement<SpeedLimit>;
  SpeedLimit(Id id, const AttributeMap& attributes, const TrafficSignsWithType& trafficSigns,
             const TrafficSignsWithType& cancellingTrafficSigns = {}, const LineStrings3d& refLines = {},
             const LineStrings3d& cancelLines = {});
  explicit SpeedLimit(const RegulatoryElementDataPtr& data);
};
}  // namespace lanelet
