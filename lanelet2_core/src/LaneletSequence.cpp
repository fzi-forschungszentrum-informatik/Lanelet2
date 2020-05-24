#include "lanelet2_core/primitives/LaneletSequence.h"

namespace lanelet {

lanelet::LaneletSequenceData::LaneletSequenceData(lanelet::ConstLanelets lanelets)
    : lanelets_{std::move(lanelets)},
      leftBound_{utils::transform(lanelets_, [](const auto& l) { return l.leftBound(); })},
      rightBound_{utils::transform(lanelets_, [](const auto& l) { return l.rightBound(); })} {}

lanelet::CompoundLineString3d lanelet::LaneletSequenceData::centerline() const {
  auto centerline = std::atomic_load_explicit(&centerline_, std::memory_order_acquire);
  if (!centerline) {
    centerline = std::make_shared<CompoundLineString3d>(
        utils::transform(lanelets_, [](const auto& l) { return l.centerline(); }));
    std::atomic_store_explicit(&centerline_, centerline, std::memory_order_release);
  }
  return *centerline;
}

lanelet::CompoundPolygon3d lanelet::LaneletSequenceData::polygon() const {
  auto polygon = std::atomic_load_explicit(&polygon_, std::memory_order_acquire);
  if (!polygon) {
    polygon = std::make_shared<CompoundPolygon3d>(CompoundLineStrings3d{leftBound_, rightBound_.invert()});
    std::atomic_store_explicit(&polygon_, polygon, std::memory_order_release);
  }
  return *polygon;
}

RegulatoryElementConstPtrs LaneletSequence::regulatoryElements() const {
  return utils::concatenate(lanelets(), [](const auto& elem) { return elem.regulatoryElements(); });
}

}  // namespace lanelet
