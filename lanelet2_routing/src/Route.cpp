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
  point.setAttribute("init_lane_id", element->initLaneId());
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

LaneletSequence Route::fullLane(const ConstLanelet& ll) {
  auto element = elements_.find(ll);
  if (element == elements_.end()) {
    return LaneletSequence();
  };
  auto storedLane = lanes_.find(element->second->laneId());
  if (storedLane != lanes_.end()) {
    // If this is a circular one we want it to start at 'll'. Otherwise it will start at the lanelet we queried
    // first an will be saved.
    if (checkCircularLanes(storedLane->second)) {
      auto it = std::find(storedLane->second.begin(), storedLane->second.end(), ll);
      ConstLanelets circular(it, storedLane->second.end());
      circular.insert(std::end(circular), std::begin(storedLane->second), it);
      return LaneletSequence(circular);
    }
    return LaneletSequence(storedLane->second);
  }
  ConstLanelets lane;
  RouteElement* firstInInitLane = firstElementsInInitLane_.find(element->second->initLaneId())->second;
  RouteElement* firstInLaneStart = firstInInitLane;
  while (true) {
    const auto previous = firstInInitLane->previous();
    if (previous.size() == 1 && previous.begin()->routeElement->laneId() == firstInLaneStart->laneId()) {
      if (previous.begin()->routeElement != firstInLaneStart) {
        firstInInitLane = firstElementsInInitLane_.find(previous.begin()->routeElement->initLaneId())->second;
      } else {
        firstInInitLane = firstInLaneStart;
        break;
      }
    } else {
      break;
    }
  }

  lane.emplace_back(firstInInitLane->lanelet());
  RouteElement* start = firstInInitLane;
  while (true) {
    auto next = firstInInitLane->following();
    if (next.size() == 1 && next.begin()->relationType == RelationType::Successor) {
      if (next.begin()->routeElement != start) {
        lane.emplace_back(next.begin()->routeElement->lanelet());
        firstInInitLane = next.begin()->routeElement;
      } else {
        break;
      }
    } else {
      break;
    }
  }
  lanes_.emplace(element->second->laneId(), lane);
  return LaneletSequence(std::move(lane));
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
      addRelation(lineStringMap, pointMap, el.second, following);
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
      addRelation(lineStringMap, pointMap, el.second, conflicting);
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

LaneletRelations Route::followingRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  const auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    const auto following{element->second->following()};
    result = transformRelations(following);
  }
  return result;
}  // namespace routing

LaneletRelations Route::previousRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  const auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    const auto previous{element->second->previous()};
    result = transformRelations(previous);
  }
  return result;
}  // namespace lanelet

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
  ConstLanelets result;
  auto element = elements_.find(lanelet);
  if (element != elements_.end()) {
    return utils::transform(element->second->conflictingInRoute(),
                            [](auto& relation) { return relation.routeElement->lanelet(); });
  }
  return result;
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
      if (follower.routeElement->id() == el.first.id()) {
        // Most probably a pointer is invalidated rather than two lanelets actually have the same ID.
        errors.emplace_back("Left element of " + std::to_string(el.first.id()) + " has the same ID");
      }
      auto previous{follower.routeElement->previous()};
      if (!utils::anyOf(previous, [&el](const auto& prev) { return prev.routeElement == el.second.get(); })) {
        errors.emplace_back("Element " + std::to_string(el.first.id()) + " has a 'succeeding' relation to " +
                            std::to_string(follower.routeElement->id()) + " but there is no relation back");
      }
    }
    if (following.size() == 1 && following.front().relationType == RelationType::Successor &&
        el.second->laneId() != following.front().routeElement->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and lanelet " +
                          std::to_string(following.front().routeElement->id()) +
                          " is succeeding, but they have different lane ids " + std::to_string(el.second->laneId()) +
                          " and " + std::to_string(following.front().routeElement->laneId()));
    } else if (following.size() > 1 && el.second->laneId() == following.front().routeElement->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and at least lanelet " +
                          std::to_string(following.front().routeElement->id()) +
                          " should be diverging, but they have the same lane id " +
                          std::to_string(el.second->laneId()));
    }
    const auto previous{el.second->previous()};
    for (const auto& prev : previous) {
      if (prev.routeElement->id() == el.first.id()) {
        // Most probably a pointer is invalidated rather than two lanelets actually have the same ID.
        errors.emplace_back("Left element of " + std::to_string(el.first.id()) + " has the same ID");
      }
      auto following{prev.routeElement->following()};
      if (!utils::anyOf(following, [&el](const auto& follower) { return follower.routeElement == el.second.get(); })) {
        errors.emplace_back("Element " + std::to_string(el.first.id()) + " has a 'previous' relation to " +
                            std::to_string(prev.routeElement->id()) + " but there is no relation back");
      }
    }
    if (previous.size() == 1 && previous.front().relationType == RelationType::Successor &&
        el.second->laneId() != previous.front().routeElement->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and lanelet " +
                          std::to_string(previous.front().routeElement->id()) +
                          " is succeeding, but they have different lane ids " + std::to_string(el.second->laneId()) +
                          " and " + std::to_string(previous.front().routeElement->laneId()));
    } else if (previous.size() > 1 && el.second->laneId() == previous.front().routeElement->laneId()) {
      errors.emplace_back("Relation between lanelet " + std::to_string(el.first.id()) + " and at least lanelet " +
                          std::to_string(previous.front().routeElement->id()) +
                          " should be merging, but they have the same lane id " + std::to_string(el.second->laneId()));
    }
    const auto conflicting{el.second->conflictingInRoute()};
    for (const auto& conf : conflicting) {
      if (!utils::anyOf(conf.routeElement->conflictingInRoute(),
                        [&el](auto& other) { return el.second.get() == other.routeElement; })) {
        errors.emplace_back("Element " + std::to_string(el.first.id()) + " conflicts with " +
                            std::to_string(conf.routeElement->id()) + " but not the other way round");
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
