#include "lanelet2_map_learning/MapData.h"

#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

using namespace internal;

LaneData LaneData::build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  LaneData data;
  data.initLeftBoundaries(localSubmap, localSubmapGraph);
  data.initRightBoundaries(localSubmap, localSubmapGraph);
  data.initLaneletFeatures(localSubmap, localSubmapGraph);
  data.initCompoundFeatures(localSubmap, localSubmapGraph);
  data.updateAssociatedCpdFeatureIndices();
  return data;
}

void LaneData::initLeftBoundaries(LaneletSubmapConstPtr& localSubmap,
                                  lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    if (isRoadBorder(ll.leftBound3d())) {
      LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(ll.leftBound3d().id());
      if (itRoadBd != roadBorders_.end()) {
        itRoadBd->second.addLaneletID(ll.id());
      } else {
        roadBorders_.insert(
            {ll.leftBound3d().id(), LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(),
                                                          bdTypeToEnum(ll.leftBound3d()), ll.id())});
      }
    } else {
      LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(ll.leftBound3d().id());
      if (itLaneBd != laneDividers_.end()) {
        itLaneBd->second.addLaneletID(ll.id());
      } else {
        laneDividers_.insert(
            {ll.leftBound3d().id(), LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(),
                                                          bdTypeToEnum(ll.leftBound3d()), ll.id())});
      }
    }

    Optional<ConstLanelet> leftLL = localSubmapGraph->left(ll);
    Optional<ConstLanelet> adjLeftLL = localSubmapGraph->adjacentLeft(ll);

    if (leftLL) {
      edges_.insert({ll.id(), Edge(ll.id(), leftLL->id())});
    }
  }
}

void LaneData::initRightBoundaries(LaneletSubmapConstPtr& localSubmap,
                                   lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    if (isRoadBorder(ll.rightBound3d())) {
      LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(ll.rightBound3d().id());
      if (itRoadBd != roadBorders_.end()) {
        itRoadBd->second.addLaneletID(ll.id());
      } else {
        roadBorders_.insert(
            {ll.rightBound3d().id(), LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(),
                                                           bdTypeToEnum(ll.rightBound3d()), ll.id())});
      }
    } else {
      LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(ll.rightBound3d().id());
      if (itLaneBd != laneDividers_.end()) {
        itLaneBd->second.addLaneletID(ll.id());
      } else {
        laneDividers_.insert(
            {ll.rightBound3d().id(), LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(),
                                                           bdTypeToEnum(ll.rightBound3d()), ll.id())});
      }
    }
    Optional<ConstLanelet> rightLL = localSubmapGraph->right(ll);
    Optional<ConstLanelet> adjRightLL = localSubmapGraph->adjacentRight(ll);
    if (rightLL) {
      edges_.insert({ll.id(), Edge(ll.id(), rightLL->id())});
    }
  }
}

void LaneData::initLaneletFeatures(LaneletSubmapConstPtr& localSubmap,
                                   lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    const LaneLineStringFeature& leftBoundary = getLineStringFeatFromId(ll.leftBound().id());
    const LaneLineStringFeature& rightBoundary = getLineStringFeatFromId(ll.rightBound().id());
    LaneLineStringFeature centerline{ll.centerline3d().basicLineString(), ll.centerline3d().id(),
                                     LineStringType::Centerline, ll.id()};
    laneletFeatures_.insert({ll.id(), LaneletFeature(leftBoundary, rightBoundary, centerline, ll.id())});
  }
}

// Idea: algorithm for paths that starts with LLs with no previous, splits on junctions and terminates on LLs with
// no successors
void getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
              ConstLanelet start, ConstLanelets initPath = ConstLanelets()) {
  initPath.push_back(start);
  ConstLanelets successorLLs = localSubmapGraph->following(start, false);
  while (!successorLLs.empty()) {
    for (size_t i = 1; i != successorLLs.size(); i++) {
      getPaths(localSubmapGraph, paths, successorLLs[i], initPath);
    }
    initPath.push_back(successorLLs.front());
    successorLLs = localSubmapGraph->following(successorLLs.front(), false);
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
    throw std::runtime_error("Lanelet boundary " + std::to_string(id) +
                             " is neither a road border nor a lane divider!");
  }
  return bdType;
}

const LaneLineStringFeature& LaneData::getLineStringFeatFromId(Id id) {
  LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(id);
  LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(id);
  if (itLaneBd != laneDividers_.end()) {
    return itLaneBd->second;
  } else if (itRoadBd != roadBorders_.end()) {
    return itRoadBd->second;
  } else {
    throw std::runtime_error("Lanelet boundary " + std::to_string(id) +
                             " is neither a road border nor a lane divider!");
  }
}

std::vector<CompoundElsList> LaneData::computeCompoundLeftBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.leftBound3d().id());

  compoundBorders.push_back(CompoundElsList{start.leftBound3d().id(), currType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].leftBound3d().id());
    if (currType == newType) {
      compoundBorders.back().ids.push_back(path[i].leftBound3d().id());
    } else {
      compoundBorders.push_back(CompoundElsList{path[i].leftBound3d().id(), newType});
      currType = newType;
    }
  }
  return compoundBorders;
}

std::vector<CompoundElsList> LaneData::computeCompoundRightBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.rightBound3d().id());

  compoundBorders.push_back(CompoundElsList{start.rightBound3d().id(), currType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].rightBound3d().id());
    if (currType == newType) {
      compoundBorders.back().ids.push_back(path[i].rightBound3d().id());
    } else {
      compoundBorders.push_back(CompoundElsList{path[i].rightBound3d().id(), newType});
      currType = newType;
    }
  }
  return compoundBorders;
}

CompoundLaneLineStringFeature LaneData::computeCompoundCenterline(const ConstLanelets& path) {
  LaneLineStringFeatureList compoundCenterlines;
  for (const auto& ll : path) {
    compoundCenterlines.push_back(
        LaneLineStringFeature(ll.centerline3d().basicLineString(), ll.id(), LineStringType::Centerline, ll.id()));
  }
  return CompoundLaneLineStringFeature(compoundCenterlines, LineStringType::Centerline);
}

std::map<Id, size_t>::const_iterator findFirstOccElement(const CompoundElsList& elsList,
                                                         const std::map<Id, size_t>& searchMap) {
  for (const auto& el : elsList.ids) {
    std::map<Id, size_t>::const_iterator it = searchMap.find(el);
    if (it != searchMap.end()) {
      return it;
    }
  }
  return searchMap.end();
}

void insertAndCheckNewCompoundFeatures(std::vector<CompoundElsList>& compFeats,
                                       const std::vector<CompoundElsList>& newCompFeats,
                                       std::map<Id, size_t>& elInsertIdx) {
  for (const auto& compEl : newCompFeats) {
    std::map<Id, size_t>::const_iterator firstOccIt = findFirstOccElement(compEl, elInsertIdx);
    if (firstOccIt == elInsertIdx.end()) {
      compFeats.push_back(compEl);
      for (const Id& el : compEl.ids) {
        elInsertIdx[el] = compFeats.size() - 1;
      }
    } else if ((compFeats[firstOccIt->second].ids.size() < compEl.ids.size()) &&
               compFeats[firstOccIt->second].type == compEl.type) {
      compFeats[firstOccIt->second] = compEl;
      for (const Id& el : compEl.ids) {
        elInsertIdx[el] = firstOccIt->second;
      }
    } else if (compFeats[firstOccIt->second].ids.size() != compEl.ids.size()) {
      compFeats.push_back(compEl);
      for (const Id& el : compEl.ids) {
        elInsertIdx[el] = compFeats.size() - 1;
      }
    }
  }
}

void LaneData::initCompoundFeatures(LaneletSubmapConstPtr& localSubmap,
                                    lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  std::vector<CompoundElsList> compoundedBordersAndDividers;
  std::map<Id, size_t> elInsertIdx;

  std::vector<ConstLanelets> paths;
  for (const auto& ll : localSubmap->laneletLayer) {
    ConstLanelets previousLLs = localSubmapGraph->previous(ll, false);
    ConstLanelets successorLLs = localSubmapGraph->following(ll, false);
    if (previousLLs.empty() && !successorLLs.empty()) {
      getPaths(localSubmapGraph, paths, ll);
    }
  }

  for (const auto& path : paths) {
    std::vector<CompoundElsList> compoundedLeft = computeCompoundLeftBorders(path);
    insertAndCheckNewCompoundFeatures(compoundedBordersAndDividers, compoundedLeft, elInsertIdx);
    std::vector<CompoundElsList> compoundedRight = computeCompoundRightBorders(path);
    insertAndCheckNewCompoundFeatures(compoundedBordersAndDividers, compoundedRight, elInsertIdx);
    compoundCenterlines_.push_back(computeCompoundCenterline(path));
  }
  for (const auto& compFeat : compoundedBordersAndDividers) {
    LaneLineStringFeatureList toBeCompounded;
    for (const auto& el : compFeat.ids) {
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

void LaneData::updateAssociatedCpdFeatureIndices() {
  for (size_t i = 0; i < compoundRoadBorders_.size(); i++) {
    const auto& cpdFeat = compoundRoadBorders_[i];
    for (const auto& indFeat : cpdFeat.features()) {
      for (const auto& id : indFeat.laneletIDs()) {
        associatedCpdRoadBorderIndices_[id].push_back(i);
      }
    }
  }
  for (size_t i = 0; i < compoundLaneDividers_.size(); i++) {
    const auto& cpdFeat = compoundLaneDividers_[i];
    for (const auto& indFeat : cpdFeat.features()) {
      for (const auto& id : indFeat.laneletIDs()) {
        associatedCpdLaneDividerIndices_[id].push_back(i);
      }
    }
  }
  for (size_t i = 0; i < compoundCenterlines_.size(); i++) {
    const auto& cpdFeat = compoundCenterlines_[i];
    for (const auto& indFeat : cpdFeat.features()) {
      for (const auto& id : indFeat.laneletIDs()) {
        associatedCpdCenterlineIndices_[id].push_back(i);
      }
    }
  }
}

bool LaneData::processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  bool validRoadBorders = processFeatureMap(roadBorders_, bbox, paramType, nPoints);
  bool validLaneDividers = processFeatureMap(laneDividers_, bbox, paramType, nPoints);
  bool validLaneletFeatures = processFeatureMap(laneletFeatures_, bbox, paramType, nPoints);
  bool validCompoundRoadBorders = processFeatureVec(compoundRoadBorders_, bbox, paramType, nPoints);
  bool validCompoundLaneDividers = processFeatureVec(compoundLaneDividers_, bbox, paramType, nPoints);
  bool validCompoundCenterlines = processFeatureVec(compoundCenterlines_, bbox, paramType, nPoints);

  if (validRoadBorders && validLaneDividers && validLaneletFeatures && validCompoundRoadBorders &&
      validCompoundLaneDividers && validCompoundCenterlines) {
    return true;
  } else {
    return false;
  }
}

}  // namespace map_learning
}  // namespace lanelet