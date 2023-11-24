#include "lanelet2_map_learning/MapData.h"

namespace lanelet {
namespace map_learning {

bool isRoadBorder(const ConstLineString3d& lstring) {
  Attribute type = lstring.attribute(AttributeName::Type);
  return type == AttributeValueString::RoadBorder || type == AttributeValueString::Curbstone ||
         type == AttributeValueString::Fence;
}

LaneData LaneData::build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  LaneData data;
  for (const auto& lstring : localSubmap->lineStringLayer) {
    if (isRoadBorder(lstring)) {
      data.roadBorders_.insert(
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
      data.laneDividers_.insert(
          {ll.leftBound3d().id(),
           LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(), LineStringType::Dashed)});
      data.edgeList_.push_back(LaneEdge(ll.id(), leftLL->id(), routing::RelationType::Left));
    } else if (adjLeftLL) {
      data.laneDividers_.insert(
          {ll.leftBound3d().id(),
           LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(), LineStringType::Solid)});
      data.edgeList_.push_back(LaneEdge(ll.id(), leftLL->id(), routing::RelationType::AdjacentLeft));
    } else if (ll.leftBound3d().attribute(AttributeName::Type) != AttributeValueString::Virtual) {
      data.laneDividers_.insert(
          {ll.leftBound3d().id(),
           LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(), LineStringType::Solid)});
    }
  }
  for (const auto& ll : localSubmap->laneletLayer) {
    Optional<ConstLanelet> rightLL = localSubmapGraph->right(ll);
    Optional<ConstLanelet> adjRightLL = localSubmapGraph->adjacentRight(ll);
    if (rightLL) {
      data.laneDividers_.insert(
          {ll.rightBound3d().id(),
           LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(), LineStringType::Dashed)});
      data.edgeList_.push_back(LaneEdge(ll.id(), rightLL->id(), routing::RelationType::Right));
    } else if (adjRightLL) {
      data.laneDividers_.insert(
          {ll.rightBound3d().id(),
           LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(), LineStringType::Solid)});
      data.edgeList_.push_back(LaneEdge(ll.id(), adjRightLL->id(), routing::RelationType::AdjacentRight));
    } else if (ll.rightBound3d().attribute(AttributeName::Type) != AttributeValueString::Virtual) {
      data.laneDividers_.insert(
          {ll.rightBound3d().id(),
           LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(), LineStringType::Solid)});
    }
  }
  for (const auto& ll : localSubmap->laneletLayer) {
    ConstLanelets previousLLs = localSubmapGraph->previous(ll);
    ConstLanelets followingLLs = localSubmapGraph->following(ll);

    // Idea: algorithm for paths that starts with LLs with no previous, splits on junctions and terminates on LLs with
    // no successors
  }
  return data;
}

}  // namespace map_learning
}  // namespace lanelet