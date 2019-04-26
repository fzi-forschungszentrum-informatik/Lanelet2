#include "internal/RouteBuilder.h"
#include <unordered_set>
#include "RoutingGraph.h"

namespace lanelet {
namespace routing {

namespace {
/** @brief Searches the vector of pending elements for a specific lanelet
 *  @param lanelet Lanelet to search for
 *  @param pending Vector of pending RouteElements
 *  @return Pointer to the RouteElement or uninitialized optional */
Optional<RouteElement*> searchInPending(const ConstLanelet& lanelet, const std::vector<RouteElementUPtrs>& pending) {
  for (auto const& pendingGroupIt : pending) {
    for (auto const& pendingElement : pendingGroupIt) {
      if (pendingElement->lanelet() == lanelet) {
        return pendingElement.get();
      }
    }
  }
  return {};
}

/** @brief Adds a RouteElement to the map of elements and updates 'firstInLane'
 *  @param elements Elements that are going to be part of the route
 *  @param firstInLane Map of the first element of each lane
 *  @param element New element to be added
 *  @throws InvalidObjectStateError if 'element' already existed in 'elements' */
void addToElements(std::unordered_map<ConstLanelet, RouteElementUPtr>& elements,
                   std::map<RouteElement::LaneId, RouteElement*>& firstInLane, RouteElementUPtr& element) {
  if (element->previous().empty() || element->initLaneId() != element->previous().front().routeElement->initLaneId()) {
    auto result{firstInLane.insert(std::make_pair(element->initLaneId(), element.get()))};
    if (!result.second) {
      auto hint = result.first;
      hint++;
      firstInLane.erase(result.first);
      firstInLane.insert(hint, std::make_pair(element->initLaneId(), element.get()));
    }
  }
  const auto success{elements.emplace(std::make_pair(element->lanelet(), std::move(element)))};
  if (!success.second) {
    throw InvalidObjectStateError("Lanelet " + std::to_string(success.first->first.id()) +
                                  " is already part of the route elements");
  }
}

/** @brief Assigns a new final lane ID used in the route to an initial lane ID.
 *  Initial (temporary) lane IDs are issued when creating the route. When all elements of the route are
 * determined, these can be transfered into the final lane IDs.
 * Example: If the underlying routing graph has a diverge-situation both new lanelets need to get new lane IDs when
 * creating the route. If one of them doesn't lead to the goal lanelet, that one won't be part of the route. The
 * other one would still have a new lane ID, even though (from the perspective of the route) it's a succeeding
 * relationship within the route.
 *  @param laneIdMap Map of initial to final lane IDs
 *  @param laneId Counter for final lane IDs
 *  @param firstInLane Map of initial lane IDs to their first elements
 *  @param id Initial lane ID
 *  @return Iterator to the newly created assignment */
std::unordered_map<RouteElement::LaneId, RouteElement::LaneId>::iterator assignLaneId(
    std::unordered_map<RouteElement::LaneId, RouteElement::LaneId>& initLaneIdMap, RouteElement::LaneId& laneId,
    const std::map<RouteElement::LaneId, RouteElement*>& firstInLane, RouteElement::LaneId id) {
  std::set<RouteElement::LaneId> newInitIds;
  RouteElement::LaneId newId;
  auto firstLaneElement{firstInLane.find(id)};
  assert(firstLaneElement != firstInLane.end());
  assert(firstLaneElement->second->initLaneId() == id &&
         "InitLaneId of the first element isn't the same as the current initLaneId");
  bool searching{true};
  while (searching) {
    newInitIds.insert(firstLaneElement->second->initLaneId());
    if (firstLaneElement->second->previous().size() == 1 &&
        firstLaneElement->second->previous().front().routeElement->following().size() == 1) {
      assert(firstLaneElement->second->initLaneId() !=
                 firstLaneElement->second->previous().front().routeElement->initLaneId() &&
             "The two elements should have different initLaneIds");
      firstLaneElement = firstInLane.find(firstLaneElement->second->previous().front().routeElement->initLaneId());
      if (initLaneIdMap.find(firstLaneElement->first) != initLaneIdMap.end()) {
        searching = false;
      }
    } else {
      break;
    }
  }
  if (searching) {
    newId = ++laneId;
  } else {
    newId = initLaneIdMap.find(firstLaneElement->first)->second;
  }
  for (const auto it : newInitIds) {
    auto insert{initLaneIdMap.insert(std::make_pair(it, newId))};
    assert(insert.second && "InitLaneId already existed in LaneIdMap. This is probably a bug");
  }
  return initLaneIdMap.find(id);
}

/** @brief Moves RouteElement candidates to pending candidates if they qualify or deletes them.
 *  There are two ways in which candidate RouteElements can qualify:
 *  1. they are directly reachable from the shortest route
 *  2. at least one of the candidates has a previous element
 *  @param candidates Vector of candidates that are routable among themselves
 *  @param pending All pending route elements
 *  @param checkPrevious Determines wheter candidates qualify by having previous element(s)
 *  @return Candidates that passed the test if 'returnNewElements' has been specified.
 *  @note Candidates will be erased if they don't pass the test */
RouteElementUPtrs moveCandidatesToPending(RouteElementUPtrs& candidates, std::vector<RouteElementUPtrs>& pending,
                                          bool checkPrevious = true, bool returnNewElements = false) {
  RouteElementUPtrs newPending;
  if (!checkPrevious) {
    if (returnNewElements) {
      newPending.insert(newPending.end(), std::make_move_iterator(candidates.begin()),
                        std::make_move_iterator(candidates.end()));
    } else {
      pending.emplace_back(std::move(candidates));
    }
  } else {
    bool previous{utils::anyOf(candidates, [](const auto& it) { return !it->previous().empty(); })};
    if (previous) {
      if (returnNewElements) {
        newPending.insert(newPending.end(), std::make_move_iterator(candidates.begin()),
                          std::make_move_iterator(candidates.end()));
      } else {
        pending.emplace_back(std::move(candidates));
      }
    }
  }
  candidates.clear();
  return newPending;
}

/** @brief Moves RouteElement candidates to pending candidates if they qualify or deletes them.
 *  @param candidates All potential candidates
 *  @param pending All pending route elements
 *  @param checkPrevious Determines wheter candidates qualify by having previous element(s) */
void moveCandidatesToPending(std::vector<RouteElementUPtrs>& candidates, std::vector<RouteElementUPtrs>& pending,
                             bool checkPrevious = true) {
  for (auto& candidatesIt : candidates) {
    moveCandidatesToPending(candidatesIt, pending, checkPrevious);
  }
  candidates.clear();
}

/** @brief Moves candidates to pending elements, but returns the raw pointers of all candidates that have been added
 *  @param candidates RouteElements to include to pending
 *  @param pending Vector of pending elements
 *  @param checkPrevious Perform a tests if a previous lanelet exists
 *  @return raw pointers of all elements that have been added */
RouteElementRawPtrs moveCandidatesToPendingReturnRawPtrs(RouteElementUPtrs& candidates,
                                                         std::vector<RouteElementUPtrs>& pending,
                                                         bool checkPrevious = true) {
  candidates = moveCandidatesToPending(candidates, pending, checkPrevious, true);
  RouteElementRawPtrs result;
  for (const auto& it : candidates) {
    result.emplace_back(it.get());
  }
  moveCandidatesToPending(candidates, pending, checkPrevious);
  assert(pending.back().size() == result.size());
  return result;
}

/** @brief Moves candidates to pending elements, but returns the raw pointers of all candidates that have been added
 *  @param candidates RouteElements to include to pending
 *  @param pending Vector of pending elements
 *  @param checkPrevious Perform a tests if a previous lanelet exists
 *  @return raw pointers of all elements that have been added */
std::vector<RouteElementRawPtrs> moveCandidatesToPendingReturnRawPtrs(std::vector<RouteElementUPtrs>& candidates,
                                                                      std::vector<RouteElementUPtrs>& pending,
                                                                      bool checkPrevious = true) {
  std::vector<RouteElementRawPtrs> result;
  result.reserve(candidates.size());
  for (auto& it : candidates) {
    result.emplace_back(moveCandidatesToPendingReturnRawPtrs(it, pending, checkPrevious));
  }
  return result;
}
}  // namespace

Optional<Route> RouteBuilder::getRouteFromShortestPath(const LaneletPath& path) {
  // Set up variables
  elements_.reserve(path.size());
  RouteElement::LaneId initLaneId{0};  // Initial lane IDs used when finding elements of the route
  RouteElement* previousElement = nullptr;

  // Check lanelets of the shortest path one by one
  for (auto pathIt = path.begin(); pathIt != path.end(); pathIt++) {
    Optional<RelationType> relation;
    if (pathIt != path.begin()) {
      relation = graph_.routingRelation(*(pathIt - 1), *pathIt);
      assert(!!relation && "Following lanelets in a route must have a relation");
    }

    // If the relation if left or right, these lanelets have already been handled
    if (relation == RelationType::Left || relation == RelationType::Right) {
      auto element = elements_.find(*pathIt);
      // It can happen that the new lanelet is not in 'elements' if the relation on the way back was
      // 'adjacent_left' or 'adjacent_right'. In that case it must be moved from 'pending' to 'elements'
      if (element == elements_.end()) {
        Optional<RouteElement*> besidesElement = searchInPending(*pathIt, pending_);
        assert(besidesElement);
        std::vector<RouteElement*> pendingToElements{*besidesElement};
        addPendingToElements(pendingToElements);
        element = elements_.find(*pathIt);
      }
      previousElement = element->second.get();
      continue;
    }

    std::vector<RouteElement*> elementsQueue;  // (Pending) elements that should become part of the route
    Optional<RouteElement*> thisElement = searchInPending(*pathIt, pending_);
    if (!!relation && !!thisElement) {
      // This is the case when the previous relationship was diverging and it can be found in pending
      elementsQueue.emplace_back(*thisElement);
      // We need to pull other reachable lanelets into elements since now they're not pending anymore.
      RouteElement* probeLeft = *thisElement;
      while (probeLeft->left() && probeLeft->left()->relationType == RelationType::Left &&
             probeLeft->left()->routeElement->right() &&
             probeLeft->left()->routeElement->right()->relationType == RelationType::Right) {
        elementsQueue.emplace_back(probeLeft->left()->routeElement);
        probeLeft = probeLeft->left()->routeElement;
      }
      RouteElement* probeRight = *thisElement;
      while (probeRight->right() && probeRight->right()->relationType == RelationType::Right &&
             probeRight->right()->routeElement->left() &&
             probeRight->right()->routeElement->left()->relationType == RelationType::Left) {
        elementsQueue.emplace_back(probeRight->right()->routeElement);
        probeRight = probeRight->right()->routeElement;
      }
      processMergingLanelets(elementsQueue, initLaneId, *thisElement, previousElement, relation);
      addPendingToElements(elementsQueue);
      previousElement = *thisElement;
      continue;
    }
    RouteElementUPtr newElement =
        std::make_unique<RouteElement>(RouteElement(*pathIt, initlaneIdForRoute(*pathIt, initLaneId)));
    thisElement = newElement.get();
    if (previousElement != nullptr) {
      (*thisElement)->addPrevious(previousElement);
      previousElement->addFollowing(*thisElement);
    }
    addToElements(elements_, firstInLane_, newElement);
    processMergingLanelets(elementsQueue, initLaneId, *thisElement, previousElement, relation);

    processLeftSide(initLaneId, *thisElement);
    processRightSide(initLaneId, *thisElement);
    recursiveDivergingToPending(initLaneId, *thisElement);

    addPendingToElements(elementsQueue);
    previousElement = *thisElement;
  }
  return createRoutefromElements(path);
}

/** @brief Recursively adds specified pending route elements to the elements of the route
 *  @param pending All pending elements
 *  @param pendingToElements pending elements that should become part of the route
 *  @param firstInLane Map of elements that are the first ones in each lane
 *  @param elements Elements of the route */
void RouteBuilder::addPendingToElements(std::vector<RouteElement*>& elementsQueue) {
  while (!elementsQueue.empty()) {
    RouteElement* thisElement = elementsQueue.front();
    elementsQueue.erase(elementsQueue.begin());
    for (auto pendingGroupIt = pending_.begin(); pendingGroupIt != pending_.end(); pendingGroupIt++) {
      bool found{false};
      for (const auto& pendingElement : *pendingGroupIt) {
        if (pendingElement.get() == thisElement) {
          found = true;
          break;
        }
      }
      if (found) {
        RouteElement* first = pendingGroupIt->front().get();
        if (first->left() && !first->left()->routeElement->right()) {  // pending elements are right of optimal route
          first->left()->routeElement->setRight(RouteElementRelation{
              first, *graph_.routingRelation(first->left()->routeElement->lanelet(), first->lanelet())});
        } else if (first->right() &&
                   !first->right()->routeElement->left()) {  // pending elements are left of optimal route
          first->right()->routeElement->setLeft(RouteElementRelation{
              first, *graph_.routingRelation(first->right()->routeElement->lanelet(), first->lanelet())});
        }
        // The other elements of the pending group can now be added to the elements as well
        for (auto& pendingElement : *pendingGroupIt) {
          RouteElementRelations previous{pendingElement->previous()};
          for (const auto& element : previous) {
            element.routeElement->addFollowing(pendingElement.get());
            if (elements_.find(element.routeElement->lanelet()) == elements_.end()) {
              elementsQueue.emplace_back(element.routeElement);
            }
          }
          addToElements(elements_, firstInLane_, pendingElement);
        }
        pending_.erase(pendingGroupIt);
        break;
      }
    }
  }
}

/** @brief Finds relations to previous elements and adds them to a new route element
 *  @param elements Elements of the route
 *  @param pending Pending route elements
 *  @param newElement New route element we want to find previous elements for
 *  @param directlyConnected Could the new element be reached from the shortest path
 *  @return Pending elements that will become part of the route because of the new element */
std::vector<RouteElement*> RouteBuilder::addPreviousRelations(RouteElement* newElement, bool directlyConnected) {
  std::vector<RouteElement*> pendingToRemove;
  const LaneletRelations previous{graph_.previousRelations(newElement->lanelet(), false)};
  for (auto const& previousIt : previous) {
    const auto previousElement = elements_.find(previousIt.lanelet);
    // Search in elements
    if (previousElement != elements_.end()) {
      newElement->addPrevious(previousElement->second.get());
      if (directlyConnected) {
        previousElement->second->addFollowing(newElement);
      }
      continue;
    }
    // Search in pending elements
    for (auto const& pendingIt : pending_) {
      for (auto const& pendingElement : pendingIt) {
        if (pendingElement->lanelet() == previousIt.lanelet) {
          newElement->addPrevious(pendingElement.get());
          if (directlyConnected) {
            pendingElement->addFollowing(newElement);
            pendingToRemove.emplace_back(pendingElement.get());
          }
          break;
        }
      }
    }
  }
  return pendingToRemove;
}

Optional<Route> RouteBuilder::createRoutefromElements(const LaneletPath& path) {
  const RouteElement::LaneId startId{1000};  // Final lane IDs that are going to be valid in the route. They start
                                             // at 1000 to allow easy distinction from the temporary ones
  RouteElement::LaneId laneId{startId};
  std::unordered_map<RouteElement::LaneId, RouteElement::LaneId> laneIdMap;  // Maps initial lane IDs to final lane IDs
  // Used for 'manual' creation of the laneletMap representing all lanelets of the route
  std::unordered_map<Id, Lanelet> laneletMap;
  std::unordered_map<Id, Point3d> pointMap;
  std::unordered_map<Id, LineString3d> lineStringMap;
  ConstLanelets routeLanelets;
  // Determine conflicting lanelets, check relations and assign final lane IDs
  for (auto& elementsIt : elements_) {
    const auto conf{graph_.conflicting(elementsIt.first)};
    elementsIt.second->addConflictingInMap(conf);
    for (auto const& confIt : conf) {
      if (!confIt.isLanelet()) {
        continue;
      }
      const auto element = elements_.find(*confIt.lanelet());
      if (element != elements_.end()) {
        elementsIt.second->addConflictingInRoute(element->second.get());
      }
    }
    elementsIt.second->checkFollowing();
    elementsIt.second->checkPrevious();
    auto laneIdIt{laneIdMap.find(elementsIt.second->initLaneId())};
    if (laneIdIt == laneIdMap.end()) {
      laneIdIt = assignLaneId(laneIdMap, laneId, firstInLane_, elementsIt.second->initLaneId());
    }
    elementsIt.second->setLaneId(laneIdIt->second);
    routeLanelets.push_back(elementsIt.first);
  }
  LaneletMapConstPtr routeMap = utils::createConstMap(routeLanelets, {});
  return Route(path, elements_, firstInLane_, routeMap, laneId - startId);
}

/** @brief Determines all the lanelets that are part of a diverging-situation
 *  The key point here is, that they need to overlap/be marked as CONFLICTING
 *  @param diverging First set of diverging lanelets
 *  @return A vector of individual lanes
 *  @see determineLanesImpl */
std::vector<ConstLanelets> RouteBuilder::determineDivergingLanes(const ConstLanelets& diverging) {
  return determineLanesImpl(diverging, &RoutingGraph::following);
}

/** @brief Determines all the lanelets that are part of a merging-situation
 *  The key point here is, that they need to overlap/be marked as CONFLICTING
 *  @param merging First set of merging lanelets
 *  @return A vector of individual lanes
 *  @see determineLanesImpl */
std::vector<ConstLanelets> RouteBuilder::determineMergingLanes(const ConstLanelets& merging) {
  return determineLanesImpl(merging, &RoutingGraph::previous);
}

/** @brief Implementation of a search for diverging or merging lanes
 *  @param initialSplit First splitting or merging lanelets
 *  @param nextLanelets A function to determine the next lanelets
 *  @return A vector of individual lanes */
std::vector<ConstLanelets> RouteBuilder::determineLanesImpl(
    const ConstLanelets& initialSplit,
    const std::function<ConstLanelets(const RoutingGraph&, ConstLanelet, bool)>& nextLanelets) const {
  std::unordered_set<ConstLanelet> involvedLanelets(initialSplit.begin(), initialSplit.end());
  std::vector<ConstLanelets> lanes;
  for (const auto& it : initialSplit) {
    lanes.emplace_back(ConstLanelets(1, it));
  }

  bool progress{false};
  auto lanesIt = lanes.begin();
  while (true) {
    const ConstLanelets newLanelets{nextLanelets(graph_, lanesIt->back(), false)};
    for (const auto& newIt : newLanelets) {
      if (involvedLanelets.find(newIt) == involvedLanelets.end()) {
        auto newItConflicting = graph_.conflicting(newIt);
        if (utils::anyOf(newItConflicting, [&involvedLanelets](const auto& loa) {
              return loa.isLanelet() && involvedLanelets.find(*loa.lanelet()) != involvedLanelets.end();
            })) {
          involvedLanelets.emplace(newIt);
          if (newLanelets.size() > 2) {
            lanes.emplace_back(ConstLanelets(1, newIt));
          } else {
            lanesIt->emplace_back(newIt);
          }
          progress = true;
        }
      }
    }
    // Abort condition: We didn't find any new lanelets in any lane that overlap with diverging lanelets
    if (lanesIt + 1 == lanes.end()) {
      if (progress) {
        lanesIt = lanes.begin();
        progress = false;
      } else {
        break;
      }
    } else {
      lanesIt++;
    }
  }
  return lanes;
}

/** @brief Adds diverging lanelets to the collection of pending elements
 *  @param elements RouteElements that are part of the route
 *  @param element A new RouteElement that could have diverging following lanelets
 *  @param pending All pending RouteElements
 *  @param initLaneId Counter for initial laneIds */
std::vector<RouteElementUPtrs> RouteBuilder::divergingToPending(RouteElement* element, RouteElement::LaneId& initLaneId,
                                                                const RouteElementUPtrs& currentCandidates) {
  const ConstLanelets followers{graph_.following(element->lanelet())};
  if (followers.size() < 2) {
    return {};
  }
  std::vector<RouteElementUPtrs> newPending;
  std::vector<ConstLanelets> divergingLanes{determineDivergingLanes(followers)};
  for (const auto& laneIt : divergingLanes) {
    initLaneId++;  // Diverging lanelets always form new lanes
    RouteElement::LaneId thisLaneId{initLaneId};
    for (const auto& laneElement : laneIt) {
      Optional<RouteElement*> thisElement{searchInPending(laneElement, pending_)};
      if (!thisElement) {
        RouteElementUPtr newElement{std::make_unique<RouteElement>(RouteElement(laneElement, thisLaneId))};
        thisElement = newElement.get();
        RouteElementUPtrs newElementVec;
        newElementVec.emplace_back(std::move(newElement));
        newPending.emplace_back(std::move(newElementVec));
        ConstLanelets previousLanelets{graph_.previous(laneElement)};
        if (previousLanelets.empty()) {
          throw InvalidObjectStateError(
              "Lanelet " + std::to_string(laneElement.id()) +
              " must have a previous lanelet in a diverging situation, but no lanelet is found.");
        }
        assert(previousLanelets.size() == 1 && "No support for multiple previous elements yet");
        auto previousEl = boost::make_optional(false, static_cast<RouteElement*>(nullptr));
        auto previousInElements{elements_.find(previousLanelets.front())};
        if (previousInElements != elements_.end()) {
          previousEl = previousInElements->second.get();
        } else if (utils::anyOf(currentCandidates, [&previousLanelets](auto& candidate) {
                     return candidate->lanelet() == previousLanelets.front();
                   })) {
          previousEl = std::find_if(currentCandidates.begin(), currentCandidates.end(),
                                    [&previousLanelets](const auto& candidate) {
                                      return candidate->lanelet() == previousLanelets.front();
                                    })
                           ->get();

        } else if (utils::anyOf(newPending, [&previousLanelets](const auto& candidates) {
                     return utils::anyOf(candidates, [&previousLanelets](const auto& candidate) {
                       return candidate->lanelet() == previousLanelets.front();
                     });
                   })) {
          previousEl = searchInPending(previousLanelets.front(), newPending);
        } else {
          previousEl = searchInPending(previousLanelets.front(), pending_);
          if (!previousEl) {
            throw InvalidObjectStateError("Missing predecessor route element for lanelet " +
                                          std::to_string(laneElement.id()) +
                                          ". In a diverging situation every RouteElement must have a predecessor.");
          }
        }
        assert(!!previousEl);
        newPending.back().back()->addPrevious(*previousEl);
        processLeftSide(initLaneId, *thisElement, false, true);
        processRightSide(initLaneId, *thisElement, false, true);
      }
    }
  }
  return newPending;
}

/** @brief Determines the initial laneIds within a route
 *  @param lanelet Lanelet to get an ID for
 *  @param elements RouteElements that are part of the route
 *  @param pending Pending RouteElements
 *  @param initLaneID Mutable counter for Lane IDs
 *  @return Lane ID that should be assigned to 'lanelet'
 *  @see @ref laneIdNotes */
RouteElement::LaneId RouteBuilder::initlaneIdForRoute(const ConstLanelet& lanelet, RouteElement::LaneId& initLaneId) {
  LaneletRelations previous{graph_.previousRelations(lanelet, false)};
  if (!previous.empty() && previous.front().relationType == RelationType::Successor) {
    const auto previousIt = elements_.find(previous.front().lanelet);
    if (previousIt != elements_.end()) {
      return previousIt->second->initLaneId();
    }
    for (auto const& pendingIt : pending_) {
      for (auto const& pendingElement : pendingIt) {
        if (pendingElement->lanelet() == previous.front().lanelet) {
          return pendingElement->initLaneId();
        }
      }
    }
  }
  return ++initLaneId;
}

/** @brief Checks if the relation between a shortest-path-lanelet and its predecessor is 'merging' and adds the
 other
 merging lanelet to the pending elements
 * @param elements RouteElements that are part of the route
 * @param firstInLane Map of the first RouteElements of each lane
 * @param pending All pending RouteElements
 * @param elementsQueue Pending elements that will become an element of the route
 * @param initLaneId Mutable counter for initial laneIds
 * @param thisElement Currently processed RouteElement
 * @param previousElement Previously processed RouteElement */
void RouteBuilder::processMergingLanelets(std::vector<RouteElement*>& elementsQueue, RouteElement::LaneId& initLaneId,
                                          RouteElement* thisElement, RouteElement* previousElement,
                                          const Optional<RelationType>& relation) {
  if (!relation || *relation != RelationType::Merging || previousElement == nullptr) {
    return;
  }
  ConstLanelets merging{graph_.previous(thisElement->lanelet(), false)};
  std::vector<ConstLanelets> mergingLanes{determineMergingLanes(merging)};
  assert(mergingLanes.size() >= 2);
  auto toRemove = std::remove_if(mergingLanes.begin(), mergingLanes.end(),
                                 [&previousElement](auto& it) { return it.front() == previousElement->lanelet(); });
  mergingLanes.erase(toRemove, mergingLanes.end());
  bool progress{false};
  auto lanesIt = mergingLanes.begin();
  while (!mergingLanes.empty()) {
    bool progressWithThisLane{false};
    RouteElementUPtr mergingCandidate =
        std::make_unique<RouteElement>(RouteElement(lanesIt->back(), initlaneIdForRoute(lanesIt->back(), initLaneId)));
    std::vector<RouteElement*> newPendingToElements{addPreviousRelations(mergingCandidate.get(), true)};
    elementsQueue.insert(elementsQueue.end(), newPendingToElements.begin(), newPendingToElements.end());
    if (!mergingCandidate->previous().empty()) {
      progress = true;
      progressWithThisLane = true;
      processLeftSide(initLaneId, mergingCandidate.get(), true);
      processRightSide(initLaneId, mergingCandidate.get(), true);
      if (lanesIt->size() > 1) {
        RouteElement* lastElement = mergingCandidate.get();
        for (auto it = lanesIt->end() - 2; it >= lanesIt->begin(); it--) {
          RouteElementUPtr furtherElement =
              std::make_unique<RouteElement>(RouteElement(*it, initlaneIdForRoute(*it, initLaneId)));
          processLeftSide(initLaneId, furtherElement.get(), true);
          processRightSide(initLaneId, furtherElement.get(), true);
          furtherElement->addPrevious(lastElement);
          lastElement->addFollowing(furtherElement.get());
          std::vector<RouteElement*> newPendingToElements{addPreviousRelations(furtherElement.get(), true)};
          elementsQueue.insert(elementsQueue.end(), newPendingToElements.begin(), newPendingToElements.end());
          lastElement = furtherElement.get();
          addToElements(elements_, firstInLane_, furtherElement);
        }
        lastElement->addFollowing(thisElement);
        thisElement->addPrevious(lastElement);
      } else {
        mergingCandidate->addFollowing(thisElement);
        thisElement->addPrevious(mergingCandidate.get());
      }
      addPendingToElements(elementsQueue);
      addToElements(elements_, firstInLane_, mergingCandidate);
      lanesIt = mergingLanes.erase(lanesIt);
    }
    // Abort condition: We didn't find any new lanelets in any lane that overlap with diverging lanelets
    if ((progressWithThisLane && lanesIt == mergingLanes.end()) || lanesIt + 1 == mergingLanes.end()) {
      if (progress) {
        lanesIt = mergingLanes.begin();
        progress = false;
      } else {
        break;
      }
    } else if (progressWithThisLane) {
      continue;
    } else {
      lanesIt++;
    }
  }
}

void RouteBuilder::processLeftSide(RouteElement::LaneId& initLaneId, RouteElement* thisElement, bool directRelation,
                                   bool preferPendingToElements) {
  std::vector<RouteElementUPtrs> newPendingElements;  // Just used if 'returnNewElements' is true
  std::vector<RouteElement*> elementsQueue;           // (Pending) elements that should become part of the route
  RouteElementUPtrs currentCandidates;                // Candidates are neighboring lanelets that can not reached
  // from the shortest-route-lanelet via lane changes, but could qualify because they could have predecessors that
  // could be reached. 'current candidates' holds all lanelets between two 'adjacent_*' relations
  RouteElement* closerToRoute = thisElement;
  const LaneletRelations relationsLeft{graph_.leftRelations(thisElement->lanelet())};
  bool directLeft{
      directRelation};        // Saves wheter this lanelet can be reached from the shortest route using lane changes
  bool directLeftBack{true};  // Saves wheter the shortest route can be reached using lane changes
  for (auto const& leftIt : relationsLeft) {
    RouteElementUPtr newElement;
    Optional<RelationType> relationBack(graph_.routingRelation(leftIt.lanelet, closerToRoute->lanelet()));
    assert(!!relationBack);  // NOLINT
    if (relationBack == RelationType::AdjacentRight) {
      directLeftBack = false;
    }
    if (leftIt.relationType == RelationType::AdjacentLeft || !directLeftBack) {
      if (leftIt.relationType == RelationType::AdjacentLeft) {
        directLeft = false;
      }
      if (!currentCandidates.empty() && leftIt.relationType == RelationType::AdjacentLeft) {
        RouteElementUPtrs newPendingTemp{moveCandidatesToPending(currentCandidates, pending_, true, true)};
        if (newPendingTemp.empty()) {
          closerToRoute = nullptr;
        }
        RouteElementRawPtrs newPendingRawPtrs{moveCandidatesToPendingReturnRawPtrs(newPendingTemp, pending_, false)};
        for (auto& it : newPendingRawPtrs) {
          std::vector<RouteElementUPtrs> newDiverging{divergingToPending(it, initLaneId)};
          moveCandidatesToPending(newDiverging, pending_, false);
        }
      }
      newElement =
          std::make_unique<RouteElement>(RouteElement(leftIt.lanelet, initlaneIdForRoute(leftIt.lanelet, initLaneId)));
      if (closerToRoute != nullptr && !preferPendingToElements) {
        newElement->setRight(RouteElementRelation{closerToRoute, *relationBack});
        if (leftIt.relationType == RelationType::Left && *relationBack != RelationType::AdjacentRight) {
          closerToRoute->setLeft(RouteElementRelation{newElement.get(), RelationType::Left});
        }
      }
      closerToRoute = newElement.get();
      addPreviousRelations(newElement.get(), false);
      if (!newElement->previous().empty() && directLeftBack) {
        elementsQueue.emplace_back(newElement.get());
      }
      currentCandidates.emplace_back(std::move(newElement));
    } else if (leftIt.relationType == RelationType::Left) {
      newElement =
          std::make_unique<RouteElement>(RouteElement(leftIt.lanelet, initlaneIdForRoute(leftIt.lanelet, initLaneId)));
      newElement->setRight(RouteElementRelation{closerToRoute, *relationBack});
      std::vector<RouteElement*> newPendingToElements{addPreviousRelations(newElement.get(), directLeft)};
      if (*relationBack != RelationType::AdjacentRight) {
        closerToRoute->setLeft(RouteElementRelation{newElement.get(), RelationType::Left});
      }
      if (directLeft && directLeftBack) {
        closerToRoute = newElement.get();
        addToElements(elements_, firstInLane_, newElement);
        elementsQueue.insert(elementsQueue.end(), newPendingToElements.begin(), newPendingToElements.end());
      } else {
        closerToRoute = newElement.get();
        currentCandidates.emplace_back(std::move(newElement));
      }
    } else {
      assert(false && "Unexpected relationship between lanelets");
    }
  }
  if (!currentCandidates.empty()) {
    RouteElementUPtrs newPendingTemp;
    if (preferPendingToElements) {
      std::move(currentCandidates.begin(), currentCandidates.end(), std::back_inserter(newPendingTemp));
    } else {
      RouteElementUPtrs newPendingTempTemp{moveCandidatesToPending(currentCandidates, pending_, !directLeft, true)};
      std::move(newPendingTempTemp.begin(), newPendingTempTemp.end(), std::back_inserter(newPendingTemp));
    }
    for (auto& it : newPendingTemp) {
      std::vector<RouteElementUPtrs> newDiverging{divergingToPending(it.get(), initLaneId, newPendingTemp)};
      moveCandidatesToPending(newDiverging, pending_, false);
    }
    pending_.emplace_back(std::move(newPendingTemp));
  }
  addPendingToElements(elementsQueue);
}

void RouteBuilder::processRightSide(RouteElement::LaneId& initLaneId, RouteElement* thisElement, bool directRelation,
                                    bool preferPendingToElements) {
  std::vector<RouteElementUPtrs> newPendingElements;  // Just used if 'returnNewElements' is true
  std::vector<RouteElement*> elementsQueue;           // (Pending) elements that should become part of the route
  RouteElementUPtrs currentCandidates;                // Candidates are neighboring lanelets that can not reached
  RouteElement* closerToRoute = thisElement;
  const LaneletRelations relationsRight{graph_.rightRelations(thisElement->lanelet())};
  bool directRight{directRelation};
  bool directRightBack{true};
  for (auto const& rightIt : relationsRight) {
    RouteElementUPtr newElement;
    Optional<RelationType> relationBack{graph_.routingRelation(rightIt.lanelet, closerToRoute->lanelet())};
    assert(!!relationBack);
    if (relationBack == RelationType::AdjacentLeft) {
      directRightBack = false;
    }
    if (rightIt.relationType == RelationType::AdjacentRight || !directRightBack) {
      if (rightIt.relationType == RelationType::AdjacentRight) {
        directRight = false;
      }
      if (!currentCandidates.empty() && rightIt.relationType == RelationType::AdjacentRight) {
        RouteElementUPtrs newPendingTemp{moveCandidatesToPending(currentCandidates, pending_, true, true)};
        if (newPendingTemp.empty()) {
          closerToRoute = nullptr;
        }
        RouteElementRawPtrs newPendingRawPtrs{moveCandidatesToPendingReturnRawPtrs(newPendingTemp, pending_, false)};
        for (auto& it : newPendingRawPtrs) {
          std::vector<RouteElementUPtrs> newDiverging{divergingToPending(it, initLaneId)};
          moveCandidatesToPending(newDiverging, pending_, false);
        }
      }
      newElement = std::make_unique<RouteElement>(
          RouteElement(rightIt.lanelet, initlaneIdForRoute(rightIt.lanelet, initLaneId)));
      if (closerToRoute != nullptr && !preferPendingToElements) {
        newElement->setLeft(RouteElementRelation{closerToRoute, *relationBack});
        if (rightIt.relationType == RelationType::Right && *relationBack != RelationType::AdjacentLeft) {
          closerToRoute->setRight(RouteElementRelation{newElement.get(), RelationType::Right});
        }
      }
      closerToRoute = newElement.get();
      addPreviousRelations(newElement.get(), false);
      if (!newElement->previous().empty() && directRightBack) {
        elementsQueue.emplace_back(newElement.get());
      }
      currentCandidates.emplace_back(std::move(newElement));
    } else if (rightIt.relationType == RelationType::Right) {
      newElement = std::make_unique<RouteElement>(
          RouteElement(rightIt.lanelet, initlaneIdForRoute(rightIt.lanelet, initLaneId)));
      newElement->setLeft(RouteElementRelation{closerToRoute, *relationBack});
      std::vector<RouteElement*> newPendingToElements{addPreviousRelations(newElement.get(), directRight)};
      if (*relationBack != RelationType::AdjacentLeft) {
        closerToRoute->setRight(RouteElementRelation{newElement.get(), RelationType::Right});
      }
      if (directRight && directRightBack && !preferPendingToElements) {
        closerToRoute = newElement.get();
        addToElements(elements_, firstInLane_, newElement);
        elementsQueue.insert(elementsQueue.end(), newPendingToElements.begin(), newPendingToElements.end());
      } else {
        closerToRoute = newElement.get();
        currentCandidates.emplace_back(std::move(newElement));
      }
    } else {
      assert(false && "Unexpected relationship between lanelets");
    }
  }
  if (!currentCandidates.empty()) {
    RouteElementUPtrs newPendingTemp;
    if (preferPendingToElements) {
      std::move(currentCandidates.begin(), currentCandidates.end(), std::back_inserter(newPendingTemp));
    } else {
      RouteElementUPtrs newPendingTempTemp{moveCandidatesToPending(currentCandidates, pending_, !directRight, true)};
      std::move(newPendingTempTemp.begin(), newPendingTempTemp.end(), std::back_inserter(newPendingTemp));
    }
    for (auto& it : newPendingTemp) {
      std::vector<RouteElementUPtrs> newDiverging{divergingToPending(it.get(), initLaneId, newPendingTemp)};
      moveCandidatesToPending(newDiverging, pending_, false);
    }
    pending_.emplace_back(std::move(newPendingTemp));
  }
  addPendingToElements(elementsQueue);
}

/** @brief Perform a search for diverging lanelets recursively
 *  @param elements RouteElements that are part of the route
 *  @param firstInLane Map of the first RouteElements of each lane
 *  @param pending All pending RouteElements
 *  @param initLaneId Mutable counter for initial laneIds
 *  @param thisElement Currently processed RouteElement */
void RouteBuilder::recursiveDivergingToPending(RouteElement::LaneId& initLaneId, RouteElement* thisElement) {
  std::vector<RouteElementUPtrs> newDiverging{divergingToPending(thisElement, initLaneId)};
  std::vector<RouteElementRawPtrs> newPendingForDiverging{
      moveCandidatesToPendingReturnRawPtrs(newDiverging, pending_, false)};
  if (!newPendingForDiverging.empty() && !newPendingForDiverging.front().empty()) {
    std::vector<std::vector<RouteElementRawPtrs>> divergingQueue;
    divergingQueue.emplace_back(newPendingForDiverging);
    while (!divergingQueue.empty()) {
      for (const auto& it : divergingQueue.front()) {
        for (const auto& elementIt : it) {
          std::vector<RouteElementUPtrs> newDiverging{divergingToPending(elementIt, initLaneId)};
          auto newRawPtrs{moveCandidatesToPendingReturnRawPtrs(newDiverging, pending_, false)};
          if (!newRawPtrs.empty()) {
            divergingQueue.emplace_back(newRawPtrs);
          }
        }
      }
      divergingQueue.erase(divergingQueue.begin());
    }
  }
}
}  // namespace routing
}  // namespace lanelet
