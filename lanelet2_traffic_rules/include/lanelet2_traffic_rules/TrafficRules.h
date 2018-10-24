#pragma once
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace lanelet {
struct SpeedLimitInformation {
  Velocity speedLimit;     //!< The current speed limit (must not be Inf)
  bool isMandatory{true};  //!< False if speed limit is only a recommendation
};

enum class LaneChangeType { ToRight, ToLeft, Both, None };

class TrafficRules;
using TrafficRulesPtr = std::shared_ptr<TrafficRules>;
using TrafficRulesUPtr = std::unique_ptr<TrafficRules>;

/**
 * @brief Class for inferring traffic rules for lanelets
 *
 */
class TrafficRules {  // NOLINT
 public:
  using Configuration = std::map<std::string, Attribute>;

  /**
   * @brief Constructor
   * @param config a configuration for traffic rules. This can be necessary for
   * very specialized rule objects (ie considering the time of day or the
   * weather)
   */
  explicit TrafficRules(Configuration config = Configuration()) : config_{std::move(config)} {}
  virtual ~TrafficRules();

  /**
   * @brief returns whether it is allowed to pass/drive on this lanelet
   *
   * The result can differ by the type of the traffic participant. A sidewalk
   * lanelet is passable for a pedestrian but not for a vehicle. Orientation is
   * important. It is not possible to pass an inverted oneWay Lanelet.
   */
  virtual bool canPass(const ConstLanelet& lanelet) const;

  //! returns whether it is allowed to pass/drive on this area
  virtual bool canPass(const ConstArea& area) const;

  /**
   * @brief returns whether it is allowed to pass/drive from a lanelet to
   * another lanelet.
   *
   * The orientation of the lanelets is important. The function first checks if
   * lanelets are direcly adjacent, then checks if both lanelets are passable
   * and finally checks if any traffic rule prevents to pass between the
   * lanelets.
   */
  virtual bool canPass(const ConstLanelet& from, const ConstLanelet& to) const;
  virtual bool canPass(const ConstLanelet& from, const ConstArea& to) const;
  virtual bool canPass(const ConstArea& from, const ConstLanelet& to) const;
  virtual bool canPass(const ConstArea& from, const ConstArea& to) const;

  /**
   * @brief determines if a lane change can be made between two lanelets
   *
   * The orientation of the lanelets is important here as well.
   */
  bool canChangeLane(const ConstLanelet& from, const ConstLanelet& to) const;

  //! returns speed limit on this lanelet.
  virtual SpeedLimitInformation speedLimit(const ConstLanelet& lanelet) const = 0;

  //! returns speed limit on this area.
  virtual SpeedLimitInformation speedLimit(const ConstArea& area) const = 0;

  //! returns whether a lanelet can be driven in one direction only
  virtual bool isOneWay(const ConstLanelet& lanelet) const;

  /**
   * @brief returns whether dynamic traffic rules apply to this lanelet.
   *
   * This can be a case if e.g. a speed limit is controlled by digital signs.
   * If this returns true, additional handling must be done to find which rules
   * are dynamic and how to handle them. This makes the values returned by the
   * other functions unreliable. Handling of dynamic rules is not covered here.
   */
  virtual bool hasDynamicRules(const ConstLanelet& lanelet) const;

  /**
   * @brief checks whether traffic restrictions change between lanelets.
   */
  virtual bool hasSameTrafficRules(const ConstLanelet& lanelet, const ConstLanelet& other) const;

  virtual bool canOvertakeLeft(const ConstLanelet& lanelet) const = 0;

  virtual bool canOvertakeRight(const ConstLanelet& lanelet) const = 0;

  virtual bool rightHandTraffic() const = 0;

  /**
   * @brief configuration for this traffic rules object
   */
  const Configuration& configuration() const { return config_; }

  /**
   * @brief the traffic participant the rules are valid for (e.g. vehicle, car,
   * pedestrian, etc)
   */
  std::string participant() const;

  /**
   * @brief the the location the rules are valid for
   *
   * Should be ISO country code
   */
  std::string location() const;

 protected:
  //! implementation for canPass that checks whether it is allowed to pass from
  //! "from" to "to".
  virtual bool canPassImpl(const ConstLanelet& from, const ConstLanelet& to) const = 0;

  /**
   * @brief checks which types of lange changes are allowed along this boundary
   * to make a lane switch from one lanelet to another.
   */
  virtual LaneChangeType laneChangeType(const ConstLineString3d& boundary) const = 0;

 private:
  Configuration config_;
};

std::ostream& operator<<(std::ostream& stream, const SpeedLimitInformation& obj);

std::ostream& operator<<(std::ostream& stream, const TrafficRules& obj);

}  // namespace lanelet
