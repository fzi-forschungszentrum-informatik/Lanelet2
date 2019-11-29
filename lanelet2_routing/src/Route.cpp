#include "Route.h"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>
#include <unordered_map>
#include "Exceptions.h"
#include "RouteElement.h"

namespace lanelet {
namespace routing {

namespace {

using ConstLaneletPointMapIt = std::unordered_map<ConstLanelet, Point2d>::iterator;

/** @brief Creates a new point that represents a RouteElement in the debug lanelet map
 *  A new point at the center of gravity of the lanelet is created and added to the pointMap. Some basic information
 * like the ID is integrated in the attributes.
 *  @param pointMap Map to add the point to
 *  @param element Route element that should be represented
 *  @return Iterator to the new entry in the point map */
ConstLaneletPointMapIt createAndAddPoint(std::unordered_map<ConstLanelet, Point2d>& pointMap,
                                         const RouteElement* element) {
  ConstLanelet lanelet{element->lanelet()};
  Point2d point;
  point.setId(lanelet.id());
  point.setAttribute("id", lanelet.id());
  point.setAttribute("lane_id", element->laneId());
  boost::geometry::centroid(CompoundHybridPolygon2d(lanelet.polygon2d()), point);
  const auto emplace{pointMap.emplace(lanelet, point)};
  return emplace.first;
}

/** @brief Adds a relation between two RouteElement objects to the debug lanelet map
 *  @param lineStringMap Map of relations between lanelets
 *  @param pointMap Map of points that represent RouteElements
 *  @param from Start route element
 *  @param relations Relations that should be added */
void addRelation(std::unordered_map<std::pair<ConstLanelet, ConstLanelet>, LineString3d>& lineStringMap,
                 std::unordered_map<ConstLanelet, Point2d>& pointMap, const RouteElementUPtr& from,
                 const RouteElementRelations& relations) {
  ConstLanelet fromLanelet = from->lanelet();
  auto fromPointIt = pointMap.find(fromLanelet);
  if (fromPointIt == pointMap.end()) {
    fromPointIt = createAndAddPoint(pointMap, from.get());
  }
  for (const auto& it : relations) {
    ConstLanelet toLanelet = it.routeElement->lanelet();
    auto edgeType = it.relationType;
    std::pair<ConstLanelet, ConstLanelet> laneletPair(fromLanelet, toLanelet);
    std::pair<ConstLanelet, ConstLanelet> orderedPair(utils::orderedPair(fromLanelet, toLanelet));
    auto lineStringMapIt = lineStringMap.find(orderedPair);
    if (lineStringMapIt == lineStringMap.end()) {
      auto toPointIt = pointMap.find(toLanelet);
      if (toPointIt == pointMap.end()) {
        toPointIt = createAndAddPoint(pointMap, it.routeElement);
      }
      LineString2d lineString;
      lineString.push_back(fromPointIt->second);
      lineString.push_back(toPointIt->second);
      bool successful;
      std::tie(lineStringMapIt, successful) = lineStringMap.emplace(orderedPair, LineString3d(lineString));
      lineStringMapIt->second.setAttribute("relation_1", relationToString(edgeType));
    } else {
      std::string direction;
      lineStringMapIt->second.front().id() == fromPointIt->first.id() ? direction = "relation_"
                                                                      : direction = "relation_reverse_";
      for (size_t it = 1; it >= 1; it++) {  /// Finding the next unused attribute.
        try {
          lineStringMapIt->second.attribute(direction + std::to_string(it));
        } catch (NoSuchAttributeError&) {
          lineStringMapIt->second.setAttribute(direction + std::to_string(it), relationToString(edgeType));
          break;
        }
      }
    }
  }
}
}  // anonymous namespace

LaneletPath Route::remainingShortestPath(const ConstLanelet& ll) const {
  auto iter = std::find(shortestPath_.begin(), shortestPath_.end(), ll);
  if (iter == shortestPath_.end()) {
    return LaneletPath{};
  }
  if (!shortestPath_.empty() && shortestPath_.front() == shortestPath_.back()) {  // circular
    ConstLanelets llts{shortestPath_.begin(), shortestPath_.end()};
    llts.pop_back();
    std::rotate(llts.begin(), llts.begin() + std::distance(shortestPath_.begin(), iter), llts.end());
    return LaneletPath{llts};
  }
  return LaneletPath{ConstLanelets{iter, shortestPath_.end()}};
}

LaneletSequence Route::fullLane(const ConstLanelet& ll) const {
  // go back to the first lanelet of the lane (ie the first lanelet that has not exactly one predecessor and then call
  // remaining lane on it
  auto element = elements_.find(ll);
  if (element == elements_.end()) {
    return LaneletSequence();
  }
  RouteElement* begin = element->second.get();
  RouteElement* first = begin;
  while (first->previous().size() == 1) {
    auto& next = first->previous().front();
    if (next == begin) {
      break;  // handle looping lanelets
    }
    first = next;
  }
  return remainingLane(first->lanelet());
}

LaneletSequence Route::remainingLane(const ConstLanelet& ll) const {
  ConstLanelets lane;
  auto element = elements_.find(ll);
  if (element == elements_.end()) {
    return lane;
  }
  RouteElement* current = element->second.get();
  auto laneId = current->laneId();
  while (current->laneId() == laneId) {
    lane.emplace_back(current->lanelet());
    if (current->following().empty()) {
      break;
    }
    current = current->following().front();
    if (current == element->second.get()) {
      break;  // handle looping lanelets
    }
  }
  return lane;
}

double Route::length2d() const {
  return std::accumulate(shortestPath_.begin(), shortestPath_.end(), 0.,
                         [](double num, const ConstLanelet& ll) { return num + geometry::length2d(ll); });
}

LaneletMapPtr Route::getDebugLaneletMap() const {
  LaneletMapPtr output = std::make_shared<LaneletMap>();
  std::unordered_map<std::pair<ConstLanelet, ConstLanelet>, LineString3d> lineStringMap;
  std::unordered_map<ConstLanelet, Point2d> pointMap;
  // Create points for each element and line strings for the relations
  for (const auto& el : elements_) {
    const auto following = el.second->following();
    if (!following.empty()) {
      auto followRelation = utils::transform(following, [](auto& f) {
        return RouteElementRelation{f, RelationType::Successor};
      });
      addRelation(lineStringMap, pointMap, el.second, followRelation);
    }
    const auto left = el.second->left();
    if (left) {
      addRelation(lineStringMap, pointMap, el.second, RouteElementRelations{*left});
    }
    const auto right = el.second->right();
    if (right) {
      addRelation(lineStringMap, pointMap, el.second, RouteElementRelations{*right});
    }
    auto conflicting = el.second->conflictingInRoute();
    if (!conflicting.empty()) {
      auto confRelation = utils::transform(following, [](auto& f) {
        return RouteElementRelation{f, RelationType::Conflicting};
      });
      addRelation(lineStringMap, pointMap, el.second, confRelation);
    }
  }
  // Mark shortest path
  for (const auto& el : shortestPath_) {
    pointMap[el].setAttribute("shortest_path", true);
  }
  for (const auto& it : lineStringMap) {
    output->add(it.second);
  }  /// This could probably be replaced with the lanelet map constructor from primitives
  return output;
}

LaneletRelations transformRelations(const RouteElementRelations& relations) {
  return utils::transform(relations, [](auto& relation) { return static_cast<LaneletRelation>(relation); });
}

LaneletRelations transformRelations(const RouteElementRawPtrs& relations) {
  return utils::transform(relations, [](auto& relation) {
    return LaneletRelation{relation->lanelet(), RelationType::Successor};
  });
}
ConstLanelets transformLanelets(const RouteElementRawPtrs& relations) {
  return utils::transform(relations, [](auto& relation) { return relation->lanelet(); });
}

LaneletRelations Route::followingRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  const auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    const auto following{element->second->following()};
    result = transformRelations(following);
  }
  return result;
}

ConstLanelets Route::following(const ConstLanelet& lanelet) const {
  const auto element = elements_.find(lanelet);
  return element == elements_.end() ? ConstLanelets{} : transformLanelets(element->second->following());
}

LaneletRelations Route::previousRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  const auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    const auto previous{element->second->previous()};
    result = transformRelations(previous);
  }
  return result;
}

ConstLanelets Route::previous(const ConstLanelet& lanelet) const {
  const auto element = elements_.find(lanelet);
  return element == elements_.end() ? ConstLanelets{} : transformLanelets(element->second->previous());
}

Optional<LaneletRelation> Route::leftRelation(const ConstLanelet& lanelet) const {
  const auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    const auto left{element->second->left()};
    if (left) {
      return static_cast<LaneletRelation>(*left);
    }
  }
  return {};
}

LaneletRelations Route::leftRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    auto left = leftRelation(element->first);
    while (left) {
      result.emplace_back(*left);
      left = leftRelation(left->lanelet);
    }
  }
  return result;
}

Optional<LaneletRelation> Route::rightRelation(const ConstLanelet& lanelet) const {
  const auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    const auto right{element->second->right()};
    if (right) {
      return static_cast<LaneletRelation>(*right);
    }
  }
  return {};
}

LaneletRelations Route::rightRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    auto right = rightRelation(element->first);
    while (right) {
      result.emplace_back(*right);
      right = rightRelation(right->lanelet);
    }
  }
  return result;
}

ConstLanelets Route::conflictingInRoute(const ConstLanelet& lanelet) const {
  const auto element = elements_.find(lanelet);
  return element == elements_.end() ? ConstLanelets{} : transformLanelets(element->second->conflictingInRoute());
}

ConstLaneletOrAreas Route::conflictingInMap(const ConstLanelet& lanelet) const {
  auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    return element->second->conflictingInMap();
  }
  return {};
}

ConstLaneletOrAreas lanelet::routing::Route::allConflictingInMap() const {
  return utils::concatenateRange(elements_, [](auto& elem) {
    auto& conf = elem.second->conflictingInMap();
    return std::make_pair(std::begin(conf), std::end(conf));
  });
}

bool Route::contains(const ConstLanelet& lanelet) const { return elements_.find(lanelet) != elements_.end(); }

Route::Errors Route::checkValidity(bool throwOnError) const {
  Errors errors;
  // All elements of the shortest path are in the route
  for (const auto& ll : shortestPath_) {
    const auto element{elements_.find(ll)};
    if (element == elements_.end()) {
      errors.emplace_back("Lanelet " + std::to_string(ll.id()) +
                          " of shortest path doesn't have a corresponding RouteElement");
    }
  }
  // Check if all relations are back and forth
  for (auto const& el : elements_) {
    const auto right{el.second->right()};
    if (right && right->routeElement->id() == el.first.id()) {
      // Most probably a pointer is invalidated rather than two lanelets actually have the same ID.
      errors.emplace_back("Right element of " + std::to_string(el.first.id()) + " has the same ID");
    }
    if (right && (!right->routeElement->left() || right->routeElement->left()->routeElement != el.second.get())) {
      errors.emplace_back("Element " + std::to_string(el.first.id()) + " has a 'right' relation to " +
                          std::to_string(right->routeElement->lanelet().id()) + " but not the other way round");
    }
    const auto left{el.second->left()};
    if (left && left->routeElement->id() == el.first.id()) {
      // Most probably a pointer is invalidated rather than two lanelets actually have the same ID.
      errors.emplace_back("Left element of " + std::to_string(el.first.id()) + " has the same ID");
    }
    if (left && (!left->routeElement->right() || left->routeElement->right()->routeElement != el.second.get())) {
      errors.emplace_back("Element " + std::to_string(el.first.id()) + " has a 'left' relation to " +
                          std::to_string(left->routeElement->lanelet().id()) + " but not the other way round");
    }
    const auto following{el.second->following()};
    for (const auto& follower : following) {
      if (follower->id() == el.first.id()) {
        // Most probably a pointer is invalidated rather than two lanelets actually have the same ID.
        errors.emplace_back("Left element of " + std::to_string(el.first.id()) + " has the same ID");
      }
      if (!utils::anyOf(follower->previous(), [&el](const auto& prev) { return prev == el.second.get(); })) {
        errors.emplace_back("Element " + std::to_string(el.first.id()) + " has a 'succeeding' relation to " +
                            std::to_string(follower->id()) + " but there is no relation back");
      }
    }
    if (following.size() == 1 && el.second->laneId() != following.front()->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and lanelet " +
                          std::to_string(following.front()->id()) +
                          " is succeeding, but they have different lane ids " + std::to_string(el.second->laneId()) +
                          " and " + std::to_string(following.front()->laneId()));
    } else if (following.size() > 1 && el.second->laneId() == following.front()->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and at least lanelet " +
                          std::to_string(following.front()->id()) +
                          " should be diverging, but they have the same lane id " +
                          std::to_string(el.second->laneId()));
    }
    const auto previous{el.second->previous()};
    for (const auto& prev : previous) {
      if (prev->id() == el.first.id()) {
        // Most probably a pointer is invalidated rather than two lanelets actually have the same ID.
        errors.emplace_back("Left element of " + std::to_string(el.first.id()) + " has the same ID");
      }
      if (!utils::anyOf(prev->following(), [&el](const auto& follower) { return follower == el.second.get(); })) {
        errors.emplace_back("Element " + std::to_string(el.first.id()) + " has a 'previous' relation to " +
                            std::to_string(prev->id()) + " but there is no relation back");
      }
    }
    if (previous.size() == 1 && el.second->laneId() != previous.front()->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and lanelet " +
                          std::to_string(previous.front()->id()) + " is succeeding, but they have different lane ids " +
                          std::to_string(el.second->laneId()) + " and " + std::to_string(previous.front()->laneId()));
    } else if (previous.size() > 1 && el.second->laneId() == previous.front()->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and at least lanelet " +
                          std::to_string(previous.front()->id()) +
                          " should be merging, but they have the same lane id " + std::to_string(el.second->laneId()));
    }
    const auto conflicting{el.second->conflictingInRoute()};
    for (const auto& conf : conflicting) {
      if (!utils::anyOf(conf->conflictingInRoute(), [&el](auto& other) { return el.second.get() == other; })) {
        errors.emplace_back("Element " + std::to_string(el.first.id()) + " conflicts with " +
                            std::to_string(conf->id()) + " but not the other way round");
      }
    }
  }
  // Check if the lanelet map of elements has the same size as there are elements
  if (laneletMap_->laneletLayer.size() != elements_.size()) {
    errors.emplace_back("Lanelet layer of map has a size of " + std::to_string(laneletMap_->laneletLayer.size()) +
                        " but Elements a size of " + std::to_string(elements_.size()));
  }

  if (throwOnError && !errors.empty()) {
    std::stringstream ss;
    ss << "Errors found in routing graph:";
    for (const auto& err : errors) {
      ss << "\n\t- " << err;
    }
    throw RoutingGraphError(ss.str());
  }
  return errors;
}
}  // namespace routing
}  // namespace lanelet
