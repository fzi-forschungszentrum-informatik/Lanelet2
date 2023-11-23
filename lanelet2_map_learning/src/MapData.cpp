#include "lanelet2_map_learning/MapData.h"

namespace lanelet {
namespace map_learning {

bool isRoadBorder(const ConstLineString3d& lstring) {
  Attribute type = lstring.attribute(AttributeName::Type);
  return type == AttributeValueString::RoadBorder || type == AttributeValueString::Curbstone ||
         type == AttributeValueString::Fence;
}

LaneData::LaneData(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& lstring : localSubmap->lineStringLayer) {
    if (isRoadBorder(lstring)) {
      roadBorders_.insert(
          {lstring.id(), LaneLineStringFeature(lstring.basicLineString(), lstring.id(), LineStringType::RoadBorder)});
      continue;
    }
  }

  std::map<Id, bool> alreadyCompounded;

  // process left boundaries
  for (const auto& ll : localSubmap->laneletLayer) {
    Optional<ConstLanelet> leftLL = localSubmapGraph->left(ll);
    Optional<ConstLanelet> adjLeftLL = localSubmapGraph->adjacentLeft(ll);
    if (leftLL) {
      laneDividers_.insert(
          {ll.leftBound3d().id(),
           LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(), LineStringType::Dashed)});
      edgeList_.push_back(LaneEdge(ll.id(), leftLL->id(), routing::RelationType::Left));
    } else if (adjLeftLL) {
      laneDividers_.insert(
          {ll.leftBound3d().id(),
           LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(), LineStringType::Solid)});
      edgeList_.push_back(LaneEdge(ll.id(), leftLL->id(), routing::RelationType::AdjacentLeft));
    } else if (ll.leftBound3d().attribute(AttributeName::Type) != AttributeValueString::Virtual) {
      laneDividers_.insert(
          {ll.leftBound3d().id(),
           LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(), LineStringType::Solid)});
    }
  }
  for (const auto& ll : localSubmap->laneletLayer) {
    Optional<ConstLanelet> rightLL = localSubmapGraph->right(ll);
    Optional<ConstLanelet> adjRightLL = localSubmapGraph->adjacentRight(ll);
    if (rightLL) {
      laneDividers_.insert(
          {ll.rightBound3d().id(),
           LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(), LineStringType::Dashed)});
      edgeList_.push_back(LaneEdge(ll.id(), rightLL->id(), routing::RelationType::Right));
    } else if (adjRightLL) {
      laneDividers_.insert(
          {ll.rightBound3d().id(),
           LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(), LineStringType::Solid)});
      edgeList_.push_back(LaneEdge(ll.id(), adjRightLL->id(), routing::RelationType::AdjacentRight));
    } else if (ll.rightBound3d().attribute(AttributeName::Type) != AttributeValueString::Virtual) {
      laneDividers_.insert(
          {ll.rightBound3d().id(),
           LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(), LineStringType::Solid)});
    }
  }
  for (const auto& ll : localSubmap->laneletLayer) {
    ConstLanelets previousLLs = localSubmapGraph->previous(ll);
    ConstLanelets followingLLs = localSubmapGraph->following(ll);
  }
}

}  // namespace map_learning
}  // namespace lanelet