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
  data->initLaneletFeatures(localSubmap, localSubmapGraph);
  data->initCompoundFeatures(localSubmap, localSubmapGraph);
  data->updateAssociatedCpdFeatureIndices();
  return data;
}

void LaneData::initLeftBoundaries(LaneletSubmapConstPtr& localSubmap,
                                  lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    Id boundID = ll.leftBound3d().id();
    BasicLineString3d bound =
        ll.leftBound3d().inverted() ? ll.leftBound3d().invert().basicLineString() : ll.leftBound3d().basicLineString();
    if (isRoadBorder(ll.leftBound3d())) {
      LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(boundID);
      if (itRoadBd != roadBorders_.end()) {
        itRoadBd->second->addLaneletID(ll.id());
      } else {
        roadBorders_.insert({boundID, std::make_shared<LaneLineStringFeature>(
                                          bound, boundID, bdTypeToEnum(ll.leftBound3d()), Ids{ll.id()}, false)});
      }
    } else {
      LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(boundID);
      if (itLaneBd != laneDividers_.end()) {
        itLaneBd->second->addLaneletID(ll.id());
      } else {
        laneDividers_.insert({boundID, std::make_shared<LaneLineStringFeature>(
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
      LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(boundID);
      if (itRoadBd != roadBorders_.end()) {
        itRoadBd->second->addLaneletID(ll.id());
      } else {
        roadBorders_.insert({boundID, std::make_shared<LaneLineStringFeature>(
                                          bound, boundID, bdTypeToEnum(ll.rightBound3d()), Ids{ll.id()}, false)});
      }
    } else {
      LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(boundID);
      if (itLaneBd != laneDividers_.end()) {
        itLaneBd->second->addLaneletID(ll.id());
      } else {
        laneDividers_.insert({boundID, std::make_shared<LaneLineStringFeature>(
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

void LaneData::initLaneletFeatures(LaneletSubmapConstPtr& localSubmap,
                                   lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  for (const auto& ll : localSubmap->laneletLayer) {
    LaneLineStringFeaturePtr leftBoundary = getLineStringFeatFromId(ll.leftBound().id(), ll.leftBound().inverted());
    LaneLineStringFeaturePtr rightBoundary = getLineStringFeatFromId(ll.rightBound().id(), ll.leftBound().inverted());
    LaneLineStringFeaturePtr centerline =
        std::make_shared<LaneLineStringFeature>(ll.centerline3d().basicLineString(), ll.centerline3d().id(),
                                                LineStringType::Centerline, Ids{ll.id()}, ll.centerline3d().inverted());
    laneletFeatures_.insert(
        {ll.id(), std::make_shared<LaneletFeature>(leftBoundary, rightBoundary, centerline, ll.id())});
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
  LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(id);
  LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(id);
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

LaneLineStringFeaturePtr makeInverted(const LaneLineStringFeaturePtr& feat) {
  return std::make_shared<LaneLineStringFeature>(
      BasicLineString3d(feat->rawFeature().rbegin(), feat->rawFeature().rend()), feat->mapID(), feat->type(),
      feat->laneletIDs(), !feat->inverted());
}

LaneLineStringFeaturePtr LaneData::getLineStringFeatFromId(Id id, bool inverted) {
  LaneLineStringFeatures::iterator itRoadBd = roadBorders_.find(id);
  LaneLineStringFeatures::iterator itLaneBd = laneDividers_.find(id);
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

CompoundLaneLineStringFeaturePtr LaneData::computeCompoundCenterline(const ConstLanelets& path) {
  LaneLineStringFeatureList compoundCenterlines;
  for (const auto& ll : path) {
    compoundCenterlines.push_back(std::make_shared<LaneLineStringFeature>(ll.centerline3d().basicLineString(), ll.id(),
                                                                          LineStringType::Centerline, Ids{ll.id()},
                                                                          ll.centerline3d().inverted()));
  }
  return std::make_shared<CompoundLaneLineStringFeature>(compoundCenterlines, LineStringType::Centerline);
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
    if (previousLLs.empty()) {
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
    if (compFeat.ids.size() != compFeat.inverted.size()) {
      throw std::runtime_error("Unequal sizes of ids and inverted!");
    }
    for (size_t i = 0; i < compFeat.ids.size(); i++) {
      LaneLineStringFeaturePtr cmpdFeat = getLineStringFeatFromId(compFeat.ids[i], compFeat.inverted[i]);
      toBeCompounded.push_back(cmpdFeat);
    }
    LineStringType cmpdType = toBeCompounded.front()->type();
    if (cmpdType == LineStringType::RoadBorder) {
      compoundRoadBorders_.push_back(std::make_shared<CompoundLaneLineStringFeature>(toBeCompounded, cmpdType));
    } else {
      compoundLaneDividers_.push_back(std::make_shared<CompoundLaneLineStringFeature>(toBeCompounded, cmpdType));
    }
  }
}

void LaneData::updateAssociatedCpdFeatureIndices() {
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

bool LaneData::processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  bool validRoadBorders = processFeatures(roadBorders_, bbox, paramType, nPoints);
  bool validLaneDividers = processFeatures(laneDividers_, bbox, paramType, nPoints);
  bool validLaneletFeatures = processFeatures(laneletFeatures_, bbox, paramType, nPoints);
  bool validCompoundRoadBorders = processFeatures(compoundRoadBorders_, bbox, paramType, nPoints);
  bool validCompoundLaneDividers = processFeatures(compoundLaneDividers_, bbox, paramType, nPoints);
  bool validCompoundCenterlines = processFeatures(compoundCenterlines_, bbox, paramType, nPoints);

  if (validRoadBorders && validLaneDividers && validLaneletFeatures && validCompoundRoadBorders &&
      validCompoundLaneDividers && validCompoundCenterlines) {
    return true;
  } else {
    return false;
  }
}

LaneData::TensorFeatureData LaneData::getTensorFeatureData(bool pointsIn2d, bool ignoreBuffer) {
  if (!tfData_.has_value() || ignoreBuffer) {
    tfData_ = LaneData::TensorFeatureData();
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
      for (const auto& lString : cpdFeat->cutResampledAndTransformedFeature()) {
        tfData_->pointMatrixCpdRoadBorder_[pointMatrixIdxRb] = cpdFeat;
        pointMatrixIdxRb++;
      }
    }
    size_t pointMatrixIdxLd = 0;
    for (const auto& cpdFeat : compoundLaneDividers_) {
      for (const auto& lString : cpdFeat->cutResampledAndTransformedFeature()) {
        tfData_->pointMatrixCpdLaneDivider_[pointMatrixIdxLd] = cpdFeat;
        pointMatrixIdxLd++;
      }
    }
    size_t pointMatrixIdxCl = 0;
    for (const auto& cpdFeat : compoundCenterlines_) {
      for (const auto& lString : cpdFeat->cutResampledAndTransformedFeature()) {
        tfData_->pointMatrixCpdCenterline_[pointMatrixIdxCl] = cpdFeat;
        pointMatrixIdxCl++;
      }
    }
  }
  return tfData_.value();
}

CompoundLaneLineStringFeatureList associatedCpdFeats(Id mapId, const CompoundLaneLineStringFeatureList& featList,
                                                     const std::map<Id, std::vector<size_t>>& assoIndices) {
  CompoundLaneLineStringFeatureList assoFeats;
  for (const auto& idx : assoIndices.at(mapId)) {
    assoFeats.push_back(featList[idx]);
  }
  return assoFeats;
}

CompoundLaneLineStringFeatureList LaneData::associatedCpdRoadBorders(Id mapId) {
  return associatedCpdFeats(mapId, compoundRoadBorders_, associatedCpdRoadBorderIndices_);
}

CompoundLaneLineStringFeatureList LaneData::associatedCpdLaneDividers(Id mapId) {
  return associatedCpdFeats(mapId, compoundLaneDividers_, associatedCpdLaneDividerIndices_);
}

CompoundLaneLineStringFeatureList LaneData::associatedCpdCenterlines(Id mapId) {
  return associatedCpdFeats(mapId, compoundCenterlines_, associatedCpdCenterlineIndices_);
}

CompoundLaneLineStringFeaturePtr pointMatrixCpdFeat(
    size_t index, const std::map<size_t, CompoundLaneLineStringFeaturePtr>& assoFeats) {
  CompoundLaneLineStringFeaturePtr feat;
  try {
    feat = assoFeats.at(index);
  } catch (const std::out_of_range& e) {
    throw std::out_of_range("A point matrix with index " + std::to_string(index) + " does not exist!");
  }
  return feat;
}

CompoundLaneLineStringFeaturePtr LaneData::TensorFeatureData::pointMatrixCpdRoadBorder(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdRoadBorder_);
}

CompoundLaneLineStringFeaturePtr LaneData::TensorFeatureData::pointMatrixCpdLaneDivider(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdLaneDivider_);
}

CompoundLaneLineStringFeaturePtr LaneData::TensorFeatureData::pointMatrixCpdCenterline(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdCenterline_);
}

}  // namespace ml_converter
}  // namespace lanelet