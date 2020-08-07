#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/utility/Optional.h>

#include "lanelet2_routing/Forward.h"

namespace lanelet {
namespace routing {
//! @brief A lanelet path represents a set of lanelets that can be reached in order by either driving straight or doing
//! lane changes.
class LaneletPath {
 public:
  using iterator = ConstLanelets::iterator;              // NOLINT
  using const_iterator = ConstLanelets::const_iterator;  // NOLINT

  explicit LaneletPath(ConstLanelets lanelets = {}) : lanelets_{std::move(lanelets)} {}

  iterator begin() { return lanelets_.begin(); }
  const_iterator begin() const { return lanelets_.begin(); }
  iterator end() { return lanelets_.end(); }
  const_iterator end() const { return lanelets_.end(); }
  size_t size() const { return lanelets_.size(); }
  bool empty() const { return lanelets_.empty(); }
  const ConstLanelet& front() const { return lanelets_.front(); }
  const ConstLanelet& back() const { return lanelets_.back(); }
  const ConstLanelet& operator[](size_t idx) const {
    assert(idx < lanelets_.size());
    return lanelets_[idx];
  }

  bool operator!=(const LaneletPath& other) const { return !(*this == other); }

  bool operator==(const LaneletPath& other) const { return lanelets_ == other.lanelets_; }

  //! Returns all succeeding lanelets from the current position that can be reached without changing lanes
  LaneletSequence getRemainingLane(const_iterator laneletPosition) const;

  LaneletSequence getRemainingLane(const ConstLanelet& llt) const {
    return getRemainingLane(std::find(lanelets_.begin(), lanelets_.end(), llt));
  }

 private:
  ConstLanelets lanelets_;
};

//! Similar to LaneletPath, but can also contain areas.
class LaneletOrAreaPath {
 public:
  using iterator = ConstLaneletOrAreas::iterator;              // NOLINT
  using const_iterator = ConstLaneletOrAreas::const_iterator;  // NOLINT

  explicit LaneletOrAreaPath(ConstLaneletOrAreas lltsOrArea = {}) : laneletsOrAreas_{std::move(lltsOrArea)} {}

  iterator begin() { return laneletsOrAreas_.begin(); }
  const_iterator begin() const { return laneletsOrAreas_.begin(); }
  iterator end() { return laneletsOrAreas_.end(); }
  const_iterator end() const { return laneletsOrAreas_.end(); }
  size_t size() const { return laneletsOrAreas_.size(); }
  bool empty() const { return laneletsOrAreas_.empty(); }
  const ConstLaneletOrArea& front() const { return laneletsOrAreas_.front(); }
  const ConstLaneletOrArea& back() const { return laneletsOrAreas_.back(); }
  const ConstLaneletOrArea& operator[](size_t idx) const {
    assert(idx < laneletsOrAreas_.size());
    return laneletsOrAreas_[idx];
  }

  bool operator!=(const LaneletOrAreaPath& other) const { return !(*this == other); }

  bool operator==(const LaneletOrAreaPath& other) const { return laneletsOrAreas_ == other.laneletsOrAreas_; }

 private:
  ConstLaneletOrAreas laneletsOrAreas_;
};

/**
 * finds the Polygon containing all Lanelets and Areas in Path. All points on the polygon will be identical to points
 * of primitives in path.
 * @param path Path to merge
 * @return Polygon containing all Lanelets and Areas in Path
 */
BasicPolygon3d getEnclosingPolygon3d(const LaneletOrAreaPath& path);
}  // namespace routing
}  // namespace lanelet
