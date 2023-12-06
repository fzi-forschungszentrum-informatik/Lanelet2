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

  data.processLeftBoundaries(localSubmap, localSubmapGraph);
  data.processRightBoundaries(localSubmap, localSubmapGraph);
  data.processCompoundFeatures(localSubmap, localSubmapGraph);

  return data;
}

void LaneData::processLeftBoundaries(LaneletSubmapConstPtr& localSubmap,
                                     lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    if (isRoadBorder(ll.leftBound3d())) {
      roadBorders_.insert(
          {ll.leftBound3d().id(), LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(),
                                                        LineStringType::RoadBorder)});
    }

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
    } else if (ll.leftBound3d().attribute(AttributeName::Type) == AttributeValueString::Virtual) {
      laneDividers_.insert(
          {ll.leftBound3d().id(),
           LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(), LineStringType::Virtual)});
    }
  }
}

void LaneData::processRightBoundaries(LaneletSubmapConstPtr& localSubmap,
                                      lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    if (isRoadBorder(ll.rightBound3d())) {
      roadBorders_.insert(
          {ll.rightBound3d().id(), LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(),
                                                         LineStringType::RoadBorder)});
    }

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
    } else if (ll.rightBound3d().attribute(AttributeName::Type) == AttributeValueString::Virtual) {
      laneDividers_.insert(
          {ll.rightBound3d().id(), LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(),
                                                         LineStringType::Virtual)});
    }
  }
}

// Idea: algorithm for paths that starts with LLs with no previous, splits on junctions and terminates on LLs with
// no successors
void getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
              ConstLanelet start, ConstLanelets initPath = ConstLanelets()) {
  initPath.push_back(start);
  ConstLanelets successorLLs = localSubmapGraph->following(start, true);
  while (!successorLLs.empty()) {
    initPath.push_back(successorLLs.front());
    for (size_t i = 1; i != successorLLs.size(); i++) {
      getPaths(localSubmapGraph, paths, successorLLs[i], initPath);
    }
    successorLLs = localSubmapGraph->following(successorLLs.front(), true);
  }
  paths.push_back(initPath);
}

LineStringType LaneData::getLineStringTypeFromId(Id id) {
  LineStringType bdType;
  LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(id);
  LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(id);
  if (itLaneBd != laneDividers_.end()) {
    bdType = itLaneBd->second.type();
  } else if (itRoadBd != roadBorders_.end()) {
    bdType = itRoadBd->second.type();
  } else {
    throw std::runtime_error("Lanelet boundary is neither a road border nor a lane divider!");
  }
  return bdType;
}

LaneLineStringFeature LaneData::getLineStringFeatFromId(Id id) {
  LaneLineStringFeature lstringFeat;
  LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(id);
  LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(id);
  if (itLaneBd != laneDividers_.end()) {
    lstringFeat = itLaneBd->second;
  } else if (itRoadBd != roadBorders_.end()) {
    lstringFeat = itRoadBd->second;
  } else {
    throw std::runtime_error("Lanelet boundary is neither a road border nor a lane divider!");
  }
  return lstringFeat;
}

std::vector<std::vector<Id>> LaneData::computeCompoundLeftBorders(const ConstLanelets& path) {
  std::vector<std::vector<Id>> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.leftBound3d().id());

  compoundBorders.push_back(std::vector<Id>{start.leftBound3d().id()});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].leftBound3d().id());
    if (currType == newType) {
      compoundBorders.back().push_back(path[i].leftBound3d().id());
    } else {
      compoundBorders.push_back(std::vector<Id>{start.leftBound3d().id()});
    }
  }
  return compoundBorders;
}

std::vector<std::vector<Id>> LaneData::computeCompoundRightBorders(const ConstLanelets& path) {
  std::vector<std::vector<Id>> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.rightBound3d().id());

  compoundBorders.push_back(std::vector<Id>{start.rightBound3d().id()});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].rightBound3d().id());
    if (currType == newType) {
      compoundBorders.back().push_back(path[i].rightBound3d().id());
    } else {
      compoundBorders.push_back(std::vector<Id>{start.rightBound3d().id()});
    }
  }
  return compoundBorders;
}

CompoundLaneLineStringFeature LaneData::computeCompoundCenterline(const ConstLanelets& path) {
  LaneLineStringFeatureList compoundCenterlines;
  for (const auto& ll : path) {
    compoundCenterlines.push_back(
        LaneLineStringFeature(ll.centerline3d().basicLineString(), ll.id(), LineStringType::Centerline));
  }
  return CompoundLaneLineStringFeature(compoundCenterlines, LineStringType::Centerline);
}

std::map<Id, size_t>::const_iterator findFirstOccElement(const std::vector<Id>& els,
                                                         const std::map<Id, size_t>& searchMap) {
  for (const auto& el : els) {
    std::map<Id, size_t>::const_iterator it = searchMap.find(el);
    if (it != searchMap.end()) {
      return it;
    }
  }
  return searchMap.end();
}

void insertAndCheckNewCompoundFeatures(std::vector<std::vector<Id>>& compFeats,
                                       const std::vector<std::vector<Id>>& newCompFeats,
                                       std::map<Id, size_t>& elInsertIdx) {
  for (const auto& compEl : newCompFeats) {
    std::map<Id, size_t>::const_iterator firstOccIt = findFirstOccElement(compEl, elInsertIdx);
    if (firstOccIt == elInsertIdx.end()) {
      compFeats.push_back(compEl);
      for (const Id& el : compEl) {
        elInsertIdx[el] = compFeats.size() - 1;
      }
    } else if (compFeats[firstOccIt->second].size() < compEl.size()) {
      compFeats[firstOccIt->second] = compEl;
      for (const Id& el : compEl) {
        elInsertIdx[el] = firstOccIt->second;
      }
    }
  }
}

void LaneData::processCompoundFeatures(LaneletSubmapConstPtr& localSubmap,
                                       lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  std::vector<std::vector<Id>> compoundedBordersAndDividers;
  std::map<Id, size_t> elInsertIdx;
  for (const auto& bd : roadBorders_) {
    std::map<Id, size_t>::iterator it = elInsertIdx.find(bd.first);
    if (it == elInsertIdx.end()) {
      continue;
    }
    Optional<Id> prevEl;
    Optional<Id> succEl;
    for (const auto& bdComp : roadBorders_) {
      if (bd.first == bdComp.first) {
        continue;
      }
      if (bd.second.rawFeature().back() == bdComp.second.rawFeature().front()) {
        succEl = bdComp.first;
      }
      if (bd.second.rawFeature().front() == bdComp.second.rawFeature().back()) {
        prevEl = bdComp.first;
      }
    }
  }

  std::vector<ConstLanelets> paths;
  for (const auto& ll : localSubmap->laneletLayer) {
    ConstLanelets previousLLs = localSubmapGraph->previous(ll, true);
    ConstLanelets successorLLs = localSubmapGraph->following(ll, true);
    if (previousLLs.empty() && !successorLLs.empty()) {
      getPaths(localSubmapGraph, paths, ll);
    }
  }
  for (const auto& path : paths) {
    std::vector<std::vector<Id>> compoundedLeft = computeCompoundLeftBorders(path);
    insertAndCheckNewCompoundFeatures(compoundedBordersAndDividers, compoundedLeft, elInsertIdx);
    std::vector<std::vector<Id>> compoundedRight = computeCompoundRightBorders(path);
    insertAndCheckNewCompoundFeatures(compoundedBordersAndDividers, compoundedRight, elInsertIdx);
    compoundCenterlines_.push_back(computeCompoundCenterline(path));
  }
  for (const auto& compFeat : compoundedBordersAndDividers) {
    LaneLineStringFeatureList toBeCompounded;
    for (const auto& el : compFeat) {
      LaneLineStringFeature cmpdFeat = getLineStringFeatFromId(el);
      toBeCompounded.push_back(cmpdFeat);
    }
    LineStringType cmpdType = toBeCompounded.front().type();
    if (cmpdType == LineStringType::RoadBorder) {
      compoundRoadBorders_.push_back(CompoundLaneLineStringFeature(toBeCompounded, cmpdType));
    } else {
      compoundLaneDividers_.push_back(CompoundLaneLineStringFeature(toBeCompounded, cmpdType));
    }
  }
}

}  // namespace map_learning
}  // namespace lanelet