#include "lanelet2_ml_converter/MapData.h"

#include "lanelet2_ml_converter/Utils.h"

namespace lanelet {
namespace ml_converter {

using namespace internal;

LaneDataPtr LaneData::build(LaneletSubmapConstPtr& localSubmap,
                            lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  LaneDataPtr data = std::make_shared<LaneData>();
  data->initLeftBoundaries(localSubmap, localSubmapGraph);
  data->initRightBoundaries(localSubmap, localSubmapGraph);
  data->initLaneletInstances(localSubmap, localSubmapGraph);
  data->initCompoundInstances(localSubmap, localSubmapGraph);
  data->updateAssociatedCpdInstanceIndices();
  return data;
}

void LaneData::initLeftBoundaries(LaneletSubmapConstPtr& localSubmap,
                                  lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    Id boundID = ll.leftBound3d().id();
    BasicLineString3d bound =
        ll.leftBound3d().inverted() ? ll.leftBound3d().invert().basicLineString() : ll.leftBound3d().basicLineString();
    if (isRoadBorder(ll.leftBound3d())) {
      LaneLineStringInstances::iterator itRoadBd = roadBorders_.find(boundID);
      if (itRoadBd != roadBorders_.end()) {
        itRoadBd->second->addLaneletID(ll.id());
      } else {
        roadBorders_.insert({boundID, std::make_shared<LaneLineStringInstance>(
                                          bound, boundID, bdTypeToEnum(ll.leftBound3d()), Ids{ll.id()}, false)});
      }
    } else {
      LaneLineStringInstances::iterator itLaneBd = laneDividers_.find(boundID);
      if (itLaneBd != laneDividers_.end()) {
        itLaneBd->second->addLaneletID(ll.id());
      } else {
        laneDividers_.insert({boundID, std::make_shared<LaneLineStringInstance>(
                                           bound, boundID, bdTypeToEnum(ll.leftBound3d()), Ids{ll.id()}, false)});
      }
    }

    Optional<ConstLanelet> leftLL = localSubmapGraph->left(ll);
    Optional<ConstLanelet> adjLeftLL = localSubmapGraph->adjacentLeft(ll);

    if (leftLL) {
      edges_[ll.id()].push_back(Edge(ll.id(), leftLL->id(), true));
    }
  }
}

void LaneData::initRightBoundaries(LaneletSubmapConstPtr& localSubmap,
                                   lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    Id boundID = ll.rightBound3d().id();
    BasicLineString3d bound = ll.rightBound3d().inverted() ? ll.rightBound3d().invert().basicLineString()
                                                           : ll.rightBound3d().basicLineString();
    if (isRoadBorder(ll.rightBound3d())) {
      LaneLineStringInstances::iterator itRoadBd = roadBorders_.find(boundID);
      if (itRoadBd != roadBorders_.end()) {
        itRoadBd->second->addLaneletID(ll.id());
      } else {
        roadBorders_.insert({boundID, std::make_shared<LaneLineStringInstance>(
                                          bound, boundID, bdTypeToEnum(ll.rightBound3d()), Ids{ll.id()}, false)});
      }
    } else {
      LaneLineStringInstances::iterator itLaneBd = laneDividers_.find(boundID);
      if (itLaneBd != laneDividers_.end()) {
        itLaneBd->second->addLaneletID(ll.id());
      } else {
        laneDividers_.insert({boundID, std::make_shared<LaneLineStringInstance>(
                                           bound, boundID, bdTypeToEnum(ll.rightBound3d()), Ids{ll.id()}, false)});
      }
    }
    Optional<ConstLanelet> rightLL = localSubmapGraph->right(ll);
    Optional<ConstLanelet> adjRightLL = localSubmapGraph->adjacentRight(ll);
    if (rightLL) {
      edges_[ll.id()].push_back(Edge(ll.id(), rightLL->id(), true));
    }
  }
}

void LaneData::initLaneletInstances(LaneletSubmapConstPtr& localSubmap,
                                    lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    LaneLineStringInstancePtr leftBoundary = getLineStringFeatFromId(ll.leftBound().id(), ll.leftBound().inverted());
    LaneLineStringInstancePtr rightBoundary = getLineStringFeatFromId(ll.rightBound().id(), ll.leftBound().inverted());
    LaneLineStringInstancePtr centerline = std::make_shared<LaneLineStringInstance>(
        ll.centerline3d().basicLineString(), ll.centerline3d().id(), LineStringType::Centerline, Ids{ll.id()},
        ll.centerline3d().inverted());
    laneletInstances_.insert(
        {ll.id(), std::make_shared<LaneletInstance>(leftBoundary, rightBoundary, centerline, ll.id())});
  }
}

bool isLaneletInPath(const ConstLanelets& path, const ConstLanelet& ll) {
  for (const auto& el : path) {
    if (ll.id() == el.id()) {
      return true;
    }
  }
  return false;
}

// Idea: algorithm for paths that starts with LLs with no previous, splits on junctions and terminates on LLs with
// no successors
void LaneData::getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
                        ConstLanelet start, ConstLanelets initPath) {
  initPath.push_back(start);
  ConstLanelet current = start;
  ConstLanelets successorLLs = localSubmapGraph->following(current, false);
  while (!successorLLs.empty()) {
    for (size_t i = 1; i != successorLLs.size(); i++) {
      if (isLaneletInPath(initPath, successorLLs[i])) {
        continue;
      }
      edges_[current.id()].push_back(Edge(current.id(), successorLLs[i].id(), false));
      getPaths(localSubmapGraph, paths, successorLLs[i], initPath);
    }
    if (isLaneletInPath(initPath, successorLLs.front())) {
      continue;
    }
    initPath.push_back(successorLLs.front());
    edges_[current.id()].push_back(Edge(current.id(), successorLLs.front().id(), false));
    current = successorLLs.front();
    successorLLs = localSubmapGraph->following(current, false);
  }
  paths.push_back(initPath);
}

LineStringType LaneData::getLineStringTypeFromId(Id id) {
  LineStringType bdType;
  LaneLineStringInstances::iterator itRoadBd = roadBorders_.find(id);
  LaneLineStringInstances::iterator itLaneBd = laneDividers_.find(id);
  if (itLaneBd != laneDividers_.end()) {
    bdType = itLaneBd->second->type();
  } else if (itRoadBd != roadBorders_.end()) {
    bdType = itRoadBd->second->type();
  } else {
    throw std::runtime_error("Lanelet boundary " + std::to_string(id) +
                             " is neither a road border nor a lane divider!");
  }
  return bdType;
}

LaneLineStringInstancePtr makeInverted(const LaneLineStringInstancePtr& feat) {
  return std::make_shared<LaneLineStringInstance>(
      BasicLineString3d(feat->rawInstance().rbegin(), feat->rawInstance().rend()), feat->mapID(), feat->type(),
      feat->laneletIDs(), !feat->inverted());
}

LaneLineStringInstancePtr LaneData::getLineStringFeatFromId(Id id, bool inverted) {
  LaneLineStringInstances::iterator itRoadBd = roadBorders_.find(id);
  LaneLineStringInstances::iterator itLaneBd = laneDividers_.find(id);
  if (itLaneBd != laneDividers_.end()) {
    return (inverted == itLaneBd->second->inverted()) ? itLaneBd->second : makeInverted(itLaneBd->second);
  } else if (itRoadBd != roadBorders_.end()) {
    return (inverted == itRoadBd->second->inverted()) ? itRoadBd->second : makeInverted(itRoadBd->second);
  } else {
    throw std::runtime_error("Lanelet boundary " + std::to_string(id) +
                             " is neither a road border nor a lane divider!");
  }
}

std::vector<CompoundElsList> LaneData::computeCompoundLeftBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.leftBound3d().id());

  compoundBorders.push_back(CompoundElsList{start.leftBound3d().id(), start.leftBound3d().inverted(), currType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].leftBound3d().id());
    if (currType == newType) {
      compoundBorders.back().ids.push_back(path[i].leftBound3d().id());
      compoundBorders.back().inverted.push_back(path[i].leftBound3d().inverted());
    } else {
      compoundBorders.push_back(CompoundElsList{path[i].leftBound3d().id(), path[i].leftBound3d().inverted(), newType});
      currType = newType;
    }
  }
  return compoundBorders;
}

std::vector<CompoundElsList> LaneData::computeCompoundRightBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.rightBound3d().id());

  compoundBorders.push_back(CompoundElsList{start.rightBound3d().id(), start.rightBound3d().inverted(), currType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].rightBound3d().id());
    if (currType == newType) {
      compoundBorders.back().ids.push_back(path[i].rightBound3d().id());
      compoundBorders.back().inverted.push_back(path[i].rightBound3d().inverted());
    } else {
      compoundBorders.push_back(
          CompoundElsList{path[i].rightBound3d().id(), path[i].rightBound3d().inverted(), newType});
      currType = newType;
    }
  }
  return compoundBorders;
}

CompoundLaneLineStringInstancePtr LaneData::computeCompoundCenterline(const ConstLanelets& path) {
  LaneLineStringInstanceList compoundCenterlines;
  for (const auto& ll : path) {
    compoundCenterlines.push_back(std::make_shared<LaneLineStringInstance>(ll.centerline3d().basicLineString(), ll.id(),
                                                                           LineStringType::Centerline, Ids{ll.id()},
                                                                           ll.centerline3d().inverted()));
  }
  return std::make_shared<CompoundLaneLineStringInstance>(compoundCenterlines, LineStringType::Centerline);
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

bool hasElementNotInOther(const CompoundElsList& elsList1, const CompoundElsList& elsList2) {
  for (const auto& el : elsList1.ids) {
    if (std::find(elsList2.ids.begin(), elsList2.ids.end(), el) == elsList2.ids.end()) {
      return true;
    }
  }
  return false;
}

void insertAndCheckNewCompoundInstances(std::vector<CompoundElsList>& compFeats,
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
               compFeats[firstOccIt->second].type == compEl.type &&
               !hasElementNotInOther(compFeats[firstOccIt->second], compEl)) {
      compFeats[firstOccIt->second] = compEl;
      for (const Id& el : compEl.ids) {
        elInsertIdx[el] = firstOccIt->second;
      }
    } else if (compFeats[firstOccIt->second].type == compEl.type) {
      std::vector<Id> leftoverIds;
      std::vector<bool> leftoverInverted;
      bool lastLeftover{false};
      for (size_t i = 0; i < compEl.ids.size(); i++) {
        if (!elInsertIdx.count(compEl.ids[i])) {
          leftoverIds.push_back(compEl.ids[i]);
          leftoverInverted.push_back(compEl.inverted[i]);
          lastLeftover = true;
        } else if (lastLeftover) {
          CompoundElsList leftover(leftoverIds, leftoverInverted, compEl.type);
          compFeats.push_back(leftover);
          for (const Id& el : leftover.ids) {
            elInsertIdx[el] = compFeats.size() - 1;
          }
          leftoverIds.clear();
          leftoverInverted.clear();
          lastLeftover = false;
        }
      }
      if (lastLeftover) {
        CompoundElsList leftover(leftoverIds, leftoverInverted, compEl.type);
        compFeats.push_back(leftover);
        for (const Id& el : leftover.ids) {
          elInsertIdx[el] = compFeats.size() - 1;
        }
        leftoverIds.clear();
        leftoverInverted.clear();
        lastLeftover = false;
      }
    }
  }
}

void LaneData::initCompoundInstances(LaneletSubmapConstPtr& localSubmap,
                                     lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  std::vector<CompoundElsList> compoundedBordersAndDividers;
  std::map<Id, size_t> elInsertIdx;

  std::vector<ConstLanelets> paths;
  for (const auto& ll : localSubmap->laneletLayer) {
    ConstLanelets previousLLs = localSubmapGraph->previous(ll, false);
    ConstLanelets successorLLs = localSubmapGraph->following(ll, false);
    if (previousLLs.empty()) {
      getPaths(localSubmapGraph, paths, ll);
    }
  }

  for (const auto& path : paths) {
    std::vector<CompoundElsList> compoundedLeft = computeCompoundLeftBorders(path);
    insertAndCheckNewCompoundInstances(compoundedBordersAndDividers, compoundedLeft, elInsertIdx);
    std::vector<CompoundElsList> compoundedRight = computeCompoundRightBorders(path);
    insertAndCheckNewCompoundInstances(compoundedBordersAndDividers, compoundedRight, elInsertIdx);
    compoundCenterlines_.push_back(computeCompoundCenterline(path));
  }
  for (const auto& compFeat : compoundedBordersAndDividers) {
    LaneLineStringInstanceList toBeCompounded;
    if (compFeat.ids.size() != compFeat.inverted.size()) {
      throw std::runtime_error("Unequal sizes of ids and inverted!");
    }
    for (size_t i = 0; i < compFeat.ids.size(); i++) {
      LaneLineStringInstancePtr cmpdFeat = getLineStringFeatFromId(compFeat.ids[i], compFeat.inverted[i]);
      toBeCompounded.push_back(cmpdFeat);
    }
    LineStringType cmpdType = toBeCompounded.front()->type();
    if (cmpdType == LineStringType::RoadBorder) {
      compoundRoadBorders_.push_back(std::make_shared<CompoundLaneLineStringInstance>(toBeCompounded, cmpdType));
    } else {
      compoundLaneDividers_.push_back(std::make_shared<CompoundLaneLineStringInstance>(toBeCompounded, cmpdType));
    }
  }
}

void LaneData::updateAssociatedCpdInstanceIndices() {
  for (size_t i = 0; i < compoundRoadBorders_.size(); i++) {
    const auto& cpdFeat = compoundRoadBorders_[i];
    for (const auto& indFeat : cpdFeat->features()) {
      for (const auto& id : indFeat->laneletIDs()) {
        associatedCpdRoadBorderIndices_[id].push_back(i);
      }
      associatedCpdRoadBorderIndices_[indFeat->mapID()].push_back(i);
    }
  }
  for (size_t i = 0; i < compoundLaneDividers_.size(); i++) {
    const auto& cpdFeat = compoundLaneDividers_[i];
    for (const auto& indFeat : cpdFeat->features()) {
      for (const auto& id : indFeat->laneletIDs()) {
        associatedCpdLaneDividerIndices_[id].push_back(i);
      }
      associatedCpdLaneDividerIndices_[indFeat->mapID()].push_back(i);
    }
  }
  for (size_t i = 0; i < compoundCenterlines_.size(); i++) {
    const auto& cpdFeat = compoundCenterlines_[i];
    for (const auto& indFeat : cpdFeat->features()) {
      for (const auto& id : indFeat->laneletIDs()) {
        associatedCpdCenterlineIndices_[id].push_back(i);
      }
    }
  }
}

bool LaneData::processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints, double pitch,
                          double roll) {
  bool validRoadBorders = processInstances(roadBorders_, bbox, paramType, nPoints, pitch, roll);
  bool validLaneDividers = processInstances(laneDividers_, bbox, paramType, nPoints, pitch, roll);
  bool validLaneletInstances = processInstances(laneletInstances_, bbox, paramType, nPoints, pitch, roll);
  bool validCompoundRoadBorders = processInstances(compoundRoadBorders_, bbox, paramType, nPoints, pitch, roll);
  bool validCompoundLaneDividers = processInstances(compoundLaneDividers_, bbox, paramType, nPoints, pitch, roll);
  bool validCompoundCenterlines = processInstances(compoundCenterlines_, bbox, paramType, nPoints, pitch, roll);

  if (validRoadBorders && validLaneDividers && validLaneletInstances && validCompoundRoadBorders &&
      validCompoundLaneDividers && validCompoundCenterlines) {
    return true;
  } else {
    return false;
  }
}

LaneData::TensorInstanceData LaneData::getTensorInstanceData(bool pointsIn2d, bool ignoreBuffer) {
  if (!tfData_.has_value() || ignoreBuffer) {
    tfData_ = LaneData::TensorInstanceData();
    tfData_->uuid_ = uuid_;
    tfData_->roadBorders_ = getPointMatrices(validRoadBorders(), pointsIn2d);
    tfData_->laneDividers_ = getPointMatrices(validLaneDividers(), pointsIn2d);
    tfData_->compoundRoadBorders_ = getPointMatrices(validCompoundRoadBorders(), pointsIn2d);
    tfData_->compoundLaneDividers_ = getPointMatrices(validCompoundLaneDividers(), pointsIn2d);
    tfData_->compoundCenterlines_ = getPointMatrices(validCompoundCenterlines(), pointsIn2d);
    for (const auto& ft : validLaneDividers()) {
      tfData_->laneDividerTypes_.push_back(ft.second->typeInt());
    }
    for (const auto& ft : validCompoundLaneDividers()) {
      tfData_->compoundLaneDividerTypes_.push_back(ft->typeInt());
    }
    size_t pointMatrixIdxRb = 0;
    for (const auto& cpdFeat : compoundRoadBorders_) {
      for (const auto& lString : cpdFeat->cutResampledAndTransformedInstance()) {
        tfData_->pointMatrixCpdRoadBorder_[pointMatrixIdxRb] = cpdFeat;
        pointMatrixIdxRb++;
      }
    }
    size_t pointMatrixIdxLd = 0;
    for (const auto& cpdFeat : compoundLaneDividers_) {
      for (const auto& lString : cpdFeat->cutResampledAndTransformedInstance()) {
        tfData_->pointMatrixCpdLaneDivider_[pointMatrixIdxLd] = cpdFeat;
        pointMatrixIdxLd++;
      }
    }
    size_t pointMatrixIdxCl = 0;
    for (const auto& cpdFeat : compoundCenterlines_) {
      for (const auto& lString : cpdFeat->cutResampledAndTransformedInstance()) {
        tfData_->pointMatrixCpdCenterline_[pointMatrixIdxCl] = cpdFeat;
        pointMatrixIdxCl++;
      }
    }
  }
  return tfData_.value();
}

CompoundLaneLineStringInstanceList associatedCpdFeats(Id mapId, const CompoundLaneLineStringInstanceList& featList,
                                                      const std::map<Id, std::vector<size_t>>& assoIndices) {
  CompoundLaneLineStringInstanceList assoFeats;
  for (const auto& idx : assoIndices.at(mapId)) {
    assoFeats.push_back(featList[idx]);
  }
  return assoFeats;
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdRoadBorders(Id mapId) {
  return associatedCpdFeats(mapId, compoundRoadBorders_, associatedCpdRoadBorderIndices_);
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdLaneDividers(Id mapId) {
  return associatedCpdFeats(mapId, compoundLaneDividers_, associatedCpdLaneDividerIndices_);
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdCenterlines(Id mapId) {
  return associatedCpdFeats(mapId, compoundCenterlines_, associatedCpdCenterlineIndices_);
}

CompoundLaneLineStringInstancePtr pointMatrixCpdFeat(
    size_t index, const std::map<size_t, CompoundLaneLineStringInstancePtr>& assoFeats) {
  CompoundLaneLineStringInstancePtr feat;
  try {
    feat = assoFeats.at(index);
  } catch (const std::out_of_range& e) {
    throw std::out_of_range("A point matrix with index " + std::to_string(index) + " does not exist!");
  }
  return feat;
}

CompoundLaneLineStringInstancePtr LaneData::TensorInstanceData::pointMatrixCpdRoadBorder(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdRoadBorder_);
}

CompoundLaneLineStringInstancePtr LaneData::TensorInstanceData::pointMatrixCpdLaneDivider(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdLaneDivider_);
}

CompoundLaneLineStringInstancePtr LaneData::TensorInstanceData::pointMatrixCpdCenterline(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdCenterline_);
}

}  // namespace ml_converter
}  // namespace lanelet