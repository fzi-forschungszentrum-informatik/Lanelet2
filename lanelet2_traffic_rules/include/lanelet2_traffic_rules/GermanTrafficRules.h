#pragma once
#include "TrafficRules.h"

namespace lanelet {
class GermanVehicle : public TrafficRules {
 public:
  using TrafficRules::TrafficRules;

  SpeedLimitInformation speedLimit(const ConstLanelet& lanelet) const override;
  SpeedLimitInformation speedLimit(const ConstArea& /*area*/) const override;

  // using areas is disallowed for vehicles in normal driving mode
  bool canPass(const ConstArea& /*area*/) const override { return false; }

 protected:
  bool canPassImpl(const ConstLanelet& from, const ConstLanelet& to) const override;

  LaneChangeType laneChangeType(const ConstLineString3d& boundary) const override;

  bool canOvertakeLeft(const ConstLanelet& /*lanelet*/) const override;
  bool canOvertakeRight(const ConstLanelet& lanelet) const override;
  bool rightHandTraffic() const override { return true; }
};

class GermanPedestrian : public TrafficRules {
 public:
  using TrafficRules::TrafficRules;

  SpeedLimitInformation speedLimit(const ConstLanelet& /*lanelet*/) const override;
  SpeedLimitInformation speedLimit(const ConstArea& /*area*/) const override;

 protected:
  bool canPassImpl(const ConstLanelet& /*from*/, const ConstLanelet& /*to*/) const override { return true; }

  LaneChangeType laneChangeType(const ConstLineString3d& boundary) const override;

  // pedestrians do not overtake.
  bool canOvertakeLeft(const ConstLanelet& /*lanelet*/) const override { return false; }
  bool canOvertakeRight(const ConstLanelet& /*lanelet*/) const override { return false; }
  bool rightHandTraffic() const override { return true; }
};
}  // namespace lanelet
