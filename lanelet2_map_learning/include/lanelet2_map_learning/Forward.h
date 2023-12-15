#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>

namespace lanelet {
namespace map_learning {

/// @brief the distinction between left and right disappears here,
///        generalized to passable and not passable (adjacent)
inline int relationToInt(lanelet::routing::RelationType type) {
  switch (type) {
    case routing::RelationType::None:
      return 0;
    case routing::RelationType::Successor:
      return 1;
    case routing::RelationType::Left:
      return 2;
    case routing::RelationType::Right:
      return 2;
    case routing::RelationType::AdjacentLeft:
      return 3;
    case routing::RelationType::AdjacentRight:
      return 3;
    case routing::RelationType::Conflicting:
      throw std::runtime_error("The relation type Conflicting should not exist in the graph!");
    case routing::RelationType::Area:
      throw std::runtime_error("The relation type Area should not exist in the graph!");
  }
  return 0;  // some compilers need that
}

enum class LaneletRepresentationType;
enum class ParametrizationType;

class MapFeature;
class LineStringFeature;
class LaneLineStringFeature;
class CompoundLaneLineStringFeature;
class LaneletFeature;
class LaneData;

}  // namespace map_learning
}  // namespace lanelet

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::MapFeature& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LineStringFeature& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneLineStringFeature& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::CompoundLaneLineStringFeature& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneletFeature& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneData& feat, const unsigned int /*version*/);

}  // namespace serialization
}  // namespace boost