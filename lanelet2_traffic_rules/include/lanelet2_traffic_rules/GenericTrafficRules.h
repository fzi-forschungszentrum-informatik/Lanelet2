#pragma once
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include "lanelet2_traffic_rules/TrafficRules.h"

namespace lanelet {
namespace traffic_rules {

enum class LaneChangeType { ToRight, ToLeft, Both, None };
struct CountrySpeedLimits {
  SpeedLimitInformation vehicleUrbanRoad;
  SpeedLimitInformation vehicleNonurbanRoad;
  SpeedLimitInformation vehicleUrbanHighway;
  SpeedLimitInformation vehicleNonurbanHighway;
  SpeedLimitInformation playStreet;
  SpeedLimitInformation pedestrian;
  SpeedLimitInformation bicycle;
};

class TrafficRules;
using TrafficRulesPtr = std::shared_ptr<TrafficRules>;
using TrafficRulesUPtr = std::unique_ptr<TrafficRules>;

//! This class generically implements the TrafficRules class in the sense of the tagging specification. It is designed
//! to make sense for most countries and participants. Country specific details (traffic signs, speed limits
//! regulations, ...) must be implemented by inheriting classes.
class GenericTrafficRules : public TrafficRules {  // NOLINT
 public:
  using TrafficRules::TrafficRules;

  /**
   * @brief returns whether it is allowed to pass/drive on this lanelet
   *
   * The result can differ by the type of the traffic participant. A sidewalk
   * lanelet is passable for a pedestrian but not for a vehicle. Orientation is
   * important. It is not possible to pass an inverted oneWay Lanelet.
   */
  bool canPass(const ConstLanelet& lanelet) const override;

  //! returns whether it is allowed to pass/drive on this area
  bool canPass(const ConstArea& area) const override;

  /**
   * @brief returns whether it is allowed to pass/drive from a lanelet to
   * another lanelet.
   *
   * The orientation of the lanelets is important. The function first checks if
   * lanelets are direcly adjacent, then checks if both lanelets are passable
   * and finally checks if any traffic rule prevents to pass between the
   * lanelets.
   */
  bool canPass(const ConstLanelet& from, const ConstLanelet& to) const override;
  bool canPass(const ConstLanelet& from, const ConstArea& to) const override;
  bool canPass(const ConstArea& from, const ConstLanelet& to) const override;
  bool canPass(const ConstArea& from, const ConstArea& to) const override;

  /**
   * @brief determines if a lane change can be made between two lanelets
   *
   * The orientation of the lanelets is important here as well.
   */
  bool canChangeLane(const ConstLanelet& from, const ConstLanelet& to) const override;

  //! returns speed limit on this lanelet.
  SpeedLimitInformation speedLimit(const ConstLanelet& lanelet) const override;

  //! returns speed limit on this area.
  SpeedLimitInformation speedLimit(const ConstArea& area) const override;

  //! returns whether a lanelet can be driven in one direction only
  bool isOneWay(const ConstLanelet& lanelet) const override;

  /**
   * @brief returns whether dynamic traffic rules apply to this lanelet.
   *
   * This can be a case if e.g. a speed limit is controlled by digital signs.
   * If this returns true, additional handling must be done to find which rules
   * are dynamic and how to handle them. This makes the values returned by the
   * other functions unreliable. Handling of dynamic rules is not covered here.
   */
  bool hasDynamicRules(const ConstLanelet& lanelet) const override;

 protected:
  //! Called by canPass to check if traffic rules make a certain primitive drivable/not drivable
  //! If the optional is empty, there is no traffic rule that determines this.
  virtual Optional<bool> canPass(const RegulatoryElementConstPtrs& regElems) const = 0;

  //! Called by canPass to check if a certain lanelet type is drivable for this participant
  virtual Optional<bool> canPass(const std::string& type, const std::string& /*location*/) const;

  /**
   * @brief checks which types of lange changes are allowed along this boundary
   * to make a lane switch from one lanelet to another.
   */
  virtual LaneChangeType laneChangeType(const ConstLineString3d& boundary, bool virtualIsPassable) const;

  //! Overloads should return a descripton of their countrie's speed limit definition
  virtual const CountrySpeedLimits& countrySpeedLimits() const = 0;

  //! Returns speed limit as defined by regiualtory elements if present
  virtual Optional<SpeedLimitInformation> speedLimit(const RegulatoryElementConstPtrs& regelems) const = 0;

 private:
  SpeedLimitInformation speedLimit(const RegulatoryElementConstPtrs& regelems, const AttributeMap& attributes) const;
};
}  // namespace traffic_rules
}  // namespace lanelet
