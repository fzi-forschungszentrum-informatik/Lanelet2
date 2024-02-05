#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>

namespace lanelet {
namespace ml_converter {

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

class MapInstance;
class LineStringInstance;
class LaneLineStringInstance;
class CompoundLaneLineStringInstance;
class LaneletInstance;
class LaneData;

}  // namespace ml_converter
}  // namespace lanelet

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::MapInstance& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LineStringInstance& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LaneLineStringInstance& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::CompoundLaneLineStringInstance& feat,
               const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LaneletInstance& feat, const unsigned int /*version*/);

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LaneData& feat, const unsigned int /*version*/);

}  // namespace serialization
}  // namespace boost