#pragma once

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include "lanelet2_routing/Forward.h"

namespace lanelet {
namespace routing {

/** @brief Abstract class to define a framework for custom routing cost calculation modules.
 *  This interfaces can be implemented to allow routing cost calculation based on specific needs (e.g. road conditions).
 *  As of now, two modules are implemented which should satisfy basic needs:
 *    1. Distance-based calculation
 *    2. Travel time-based calculation
 * Routing cost modules *can* be used to prohibit a certain relation by setting the cost to infinity. Still it is
 * generally better to use regulatory elements and the TrafficRules module to achieve this. The cost can represent
 * whatever you want it to be - seconds, meter, lane changes, ... but it must not be negative! */
class RoutingCost {  // NOLINT
 public:
  virtual ~RoutingCost() = default;

  /** @brief Get the cost of the transistion from one to another lanelet
   *  @param trafficRules TrafficRules module to apply
   *  @param from The lanelet or area the traffic participant is on (assumed to be in the middle)
   *  @param to The lanelet or area the traffic participant is reaching for (reference is the middle again)
   *  @return Routing cost for passing this lanelet
      @note The calculated cost will be assigned to the edge in the graph between 'from' and 'to'. So typically this
   should represent the cost of passing the 'from'-lanelet with regards of going into the 'to'-lanelet.*/
  virtual double getCostSucceeding(const traffic_rules::TrafficRules& trafficRules, const ConstLaneletOrArea& from,
                                   const ConstLaneletOrArea& to) const = 0;

  /** @brief Get the cost of the lane change between two adjacent lanelets
   *  @param trafficRules TrafficRules module to apply
   *  @param from The lanelet or area the traffic participant is on (assumed to be in the middle)
   *  @param to The lanelet or area the traffic participant is reaching for (reference is the middle again)
   *  @return Routing cost of the lane change
      @note The calculated cost will be assigned to the edge in the graph between 'from' and 'to'. So typically this
   should represent some artificial cost for a lane change. */
  virtual double getCostLaneChange(const traffic_rules::TrafficRules& trafficRules, const ConstLanelets& from,
                                   const ConstLanelets& to) const = 0;
};

/** @brief A basic distance-based routing cost module.
 *  Uses the 2D length and a fixed lane change cost to evaluate relations. */
class RoutingCostDistance : public RoutingCost {
 public:
  //! Distance cost of a lane change [m]. If a lane change requires less than minLaneChangeLength, no lane change will
  //! be possible here. Instead, relation between the involved lanelets will be "adjacent".
  explicit RoutingCostDistance(double laneChangeCost, double minLaneChangeLength = 0.)
      : laneChangeCost_{laneChangeCost}, minChangeLength_{minLaneChangeLength} {
    if (laneChangeCost_ < 0.0) {
      throw InvalidInputError("Lane change cost must be positive, but it is " + std::to_string(laneChangeCost_));
    }
  }
  inline double getCostSucceeding(const traffic_rules::TrafficRules& /*trafficRules*/, const ConstLaneletOrArea& from,
                                  const ConstLaneletOrArea& to) const override {
    auto getLength = [this](auto& lltOrArea) -> double { return this->length(lltOrArea); };
    return (from.applyVisitor(getLength) + to.applyVisitor(getLength)) / 2;
  }
  inline double getCostLaneChange(const traffic_rules::TrafficRules& /*trafficRules*/, const ConstLanelets& from,
                                  const ConstLanelets& /*to*/) const noexcept override {
    if (minChangeLength_ <= 0.) {
      return laneChangeCost_;
    }
    auto totalLength = std::accumulate(from.begin(), from.end(), 0.,
                                       [this](double acc, auto& llt) { return acc + this->length(llt); });
    return totalLength >= minChangeLength_ ? laneChangeCost_ : std::numeric_limits<double>::infinity();
  }

 private:
  static double length(const ConstLanelet& ll) noexcept;
  static double length(const ConstArea& ar) noexcept;
  const double laneChangeCost_, minChangeLength_;  // NOLINT
};

/** @brief A basic travel time-based routing cost module.
 *  Uses maximum allowed speed or the maximum speed of the participant (what every is lower) and a fixed lane chance
 * cost. */
class RoutingCostTravelTime : public RoutingCost {
 public:
  RoutingCostTravelTime() = delete;

  //! Time cost of a lane change [s]. If the time is not sufficient for the lane change (i.e. minLaneChangeTime is not
  //! reached), no lane change will possible at this position. Instead the relation between the involved lanelets is
  //! "adjacent".
  explicit RoutingCostTravelTime(double laneChangeCost, double minLaneChangeTime = 0.)
      : laneChangeCost_{laneChangeCost}, minChangeTime_{minLaneChangeTime} {
    if (laneChangeCost_ < 0.0) {
      throw InvalidInputError("Lane change cost must be positive, but it is " + std::to_string(laneChangeCost_));
    }
  }

  inline double getCostLaneChange(const traffic_rules::TrafficRules& trafficRules, const ConstLanelets& from,
                                  const ConstLanelets& /*to*/) const noexcept override {
    if (minChangeTime_ <= 0.) {
      return laneChangeCost_;
    }
    auto changeTime = std::accumulate(from.begin(), from.end(), 0., [this, &trafficRules](double acc, auto& llt) {
      return acc + this->travelTime(trafficRules, llt);
    });
    return changeTime >= minChangeTime_ ? laneChangeCost_ : std::numeric_limits<double>::infinity();
  }
  inline double getCostSucceeding(const traffic_rules::TrafficRules& trafficRules, const ConstLaneletOrArea& from,
                                  const ConstLaneletOrArea& to) const override {
    auto getTravelTime = [&trafficRules, this](auto& lltOrArea) -> double {
      return this->travelTime(trafficRules, lltOrArea);
    };
    return (from.applyVisitor(getTravelTime) + to.applyVisitor(getTravelTime)) / 2;
  }

 private:
  static double travelTime(const traffic_rules::TrafficRules& trafficRules, const ConstLanelet& ll);
  static double travelTime(const traffic_rules::TrafficRules& trafficRules, const ConstArea& ar);
  const Velocity maxSpeed_;                      // NOLINT
  const double laneChangeCost_, minChangeTime_;  // NOLINT
};

//! Returns routing cost objects for initialization of routing graph with reasonable default values (for vehicles)
inline RoutingCostPtrs defaultRoutingCosts() {
  return RoutingCostPtrs{std::make_shared<RoutingCostDistance>(10.), std::make_shared<RoutingCostTravelTime>(5.)};
}
}  // namespace routing
}  // namespace lanelet
