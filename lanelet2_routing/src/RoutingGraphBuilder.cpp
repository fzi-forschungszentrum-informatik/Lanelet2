#include "lanelet2_routing/internal/RoutingGraphBuilder.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <unordered_map>

#include "lanelet2_routing/Exceptions.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace routing {
namespace internal {
namespace {
inline IdPair orderedIdPair(const Id id1, const Id id2) { return (id1 < id2) ? IdPair(id1, id2) : IdPair(id2, id1); }
}  // namespace

//! This class collects lane changable lanelets and combines them to a sequence of adjacent lanechangable lanelets
class LaneChangeLaneletsCollector {
  struct LaneChangeInfo {
    ConstLanelet target;
    bool visited;
  };
  using LaneChangeMap = std::unordered_map<ConstLanelet, LaneChangeInfo>;

 public:
  using LaneChangeLanelets = std::pair<ConstLanelets, ConstLanelets>;

  LaneChangeLaneletsCollector() = default;
  void add(ConstLanelet from, ConstLanelet to) {
    laneChanges_.emplace(std::move(from), LaneChangeInfo{std::move(to), false});
    currPos_ = laneChanges_.begin();
  }

  template <typename Func1, typename Func2>
  Optional<LaneChangeLanelets> getNextChangeLanelets(Func1&& prev, Func2&& next) {
    for (; currPos_ != laneChanges_.end() && currPos_->second.visited; ++currPos_) {
    }
    if (currPos_ == laneChanges_.end()) {
      return {};
    }
    return getLaneChangeLanelets(currPos_, std::forward<Func1>(prev), std::forward<Func2>(next));
  }

 private:
  template <typename Func1, typename Func2>
  LaneChangeLanelets getLaneChangeLanelets(LaneChangeMap::iterator iter, Func1&& prev, Func2&& next) {
    iter->second.visited = true;
    auto followers = getAdjacentLaneChangeLanelets(iter, std::forward<Func2>(next));
    auto predecessors = getAdjacentLaneChangeLanelets(iter, std::forward<Func1>(prev));
    std::reverse(predecessors.first.begin(), predecessors.first.end());
    std::reverse(predecessors.second.begin(), predecessors.second.end());
    std::pair<ConstLanelets, ConstLanelets> result;
    result.first = utils::concatenate({predecessors.first, ConstLanelets{iter->first}, followers.first});
    result.second = utils::concatenate({predecessors.second, ConstLanelets{iter->second.target}, followers.second});
    return result;
  }

  template <typename Func1>
  LaneChangeLanelets getAdjacentLaneChangeLanelets(LaneChangeMap::iterator iter, Func1&& adjacent) {
    std::pair<ConstLanelets, ConstLanelets> successors;
    while (true) {
      auto nextSourceLlts = adjacent(iter->first);
      auto nextTargetLlts = adjacent(iter->second.target);
      if (nextSourceLlts.size() != 1 || nextTargetLlts.size() != 1) {
        break;
      }
      ConstLanelet& nextSourceLlt = nextSourceLlts.front();
      ConstLanelet& nextTargetLlt = nextTargetLlts.front();
      iter = laneChanges_.find(nextSourceLlt);
      if (iter == laneChanges_.end() || iter->second.visited || iter->second.target != nextTargetLlt) {
        break;
      }
      iter->second.visited = true;
      successors.first.push_back(nextSourceLlt);
      successors.second.push_back(nextTargetLlt);
    }
    return successors;
  }

  LaneChangeMap laneChanges_;
  LaneChangeMap::iterator currPos_{laneChanges_.end()};
};

RoutingGraphBuilder::RoutingGraphBuilder(const traffic_rules::TrafficRules& trafficRules,
                                         const RoutingCostPtrs& routingCosts, const RoutingGraph::Configuration& config)
    : graph_{std::make_unique<RoutingGraphGraph>(routingCosts.size())},
      trafficRules_{trafficRules},
      routingCosts_{routingCosts},
      config_{config} {}

RoutingGraphUPtr RoutingGraphBuilder::build(const LaneletMapLayers& laneletMapLayers) {
  auto passableLanelets = getPassableLanelets(laneletMapLayers.laneletLayer, trafficRules_);
  auto passableAreas = getPassableAreas(laneletMapLayers.areaLayer, trafficRules_);
  auto passableMap = utils::createConstSubmap(passableLanelets, passableAreas);
  appendBidirectionalLanelets(passableLanelets);
  addLaneletsToGraph(passableLanelets);
  addAreasToGraph(passableAreas);
  addEdges(passableLanelets, passableMap->laneletLayer);
  addEdges(passableAreas, passableMap->laneletLayer, passableMap->areaLayer);
  return std::make_unique<RoutingGraph>(std::move(graph_), std::move(passableMap));
}

ConstLanelets RoutingGraphBuilder::getPassableLanelets(const LaneletLayer& lanelets,
                                                       const traffic_rules::TrafficRules& trafficRules) {
  ConstLanelets llts;
  llts.reserve(lanelets.size());
  std::copy_if(lanelets.begin(), lanelets.end(), std::back_inserter(llts),
               [&trafficRules](const ConstLanelet& llt) { return trafficRules.canPass(llt); });
  return llts;
}

ConstAreas RoutingGraphBuilder::getPassableAreas(const AreaLayer& areas,
                                                 const traffic_rules::TrafficRules& trafficRules) {
  ConstAreas ars;
  ars.reserve(areas.size());
  std::copy_if(areas.begin(), areas.end(), std::back_inserter(ars),
               [&trafficRules](const ConstArea& area) { return trafficRules.canPass(area); });
  return ars;
}

void RoutingGraphBuilder::appendBidirectionalLanelets(ConstLanelets& llts) {
  std::deque<ConstLanelet> invLanelets;
  for (auto& ll : llts) {
    if (trafficRules_.canPass(ll.invert())) {
      invLanelets.push_back(ll.invert());
      bothWaysLaneletIds_.emplace(ll.id());
    }
  }
  llts.insert(llts.end(), invLanelets.begin(), invLanelets.end());
}

void RoutingGraphBuilder::addLaneletsToGraph(ConstLanelets& llts) {
  for (auto& ll : llts) {
    graph_->addVertex(VertexInfo{ll});
    addPointsToSearchIndex(ll);
  }
}

void RoutingGraphBuilder::addAreasToGraph(ConstAreas& areas) {
  for (auto& ar : areas) {
    graph_->addVertex(VertexInfo{ar});
  }
}

void RoutingGraphBuilder::addEdges(const ConstLanelets& lanelets, const LaneletLayer& passableLanelets) {
  LaneChangeLaneletsCollector leftToRight;
  LaneChangeLaneletsCollector rightToLeft;
  // Check relations between lanelets
  for (auto const& ll : lanelets) {
    addFollowingEdges(ll);
    addSidewayEdge(rightToLeft, ll, ll.leftBound(), RelationType::AdjacentLeft);
    addSidewayEdge(leftToRight, ll, ll.rightBound(), RelationType::AdjacentRight);
    addConflictingEdge(ll, passableLanelets);
  }

  // now process the lane changes
  addLaneChangeEdges(rightToLeft, RelationType::Left);
  addLaneChangeEdges(leftToRight, RelationType::Right);
}

void RoutingGraphBuilder::addEdges(const ConstAreas& areas, const LaneletLayer& passableLanelets,
                                   const AreaLayer& passableAreas) {
  for (const auto& area : areas) {
    addAreaEdge(area, passableLanelets);
    addAreaEdge(area, passableAreas);
  }
}

void RoutingGraphBuilder::addFollowingEdges(const ConstLanelet& ll) {
  auto endPointsLanelets =
      pointsToLanelets_.equal_range(orderedIdPair(ll.leftBound().back().id(), ll.rightBound().back().id()));
  // Following
  ConstLanelets following;
  std::for_each(endPointsLanelets.first, endPointsLanelets.second, [&ll, this, &following](auto it) {
    if (geometry::follows(ll, it.second) && this->trafficRules_.canPass(ll, it.second)) {
      following.push_back(it.second);
    }
  });
  if (following.empty()) {
    return;
  }
  // find out if there are several previous merging lanelets
  ConstLanelets merging;
  std::for_each(endPointsLanelets.first, endPointsLanelets.second, [&following, this, &merging](auto it) {
    if (geometry::follows(it.second, following.front()) && this->trafficRules_.canPass(it.second, following.front())) {
      merging.push_back(it.second);
    }
  });
  RelationType relation = RelationType::Successor;
  for (auto& followingIt : following) {
    assignCosts(ll, followingIt, relation);
  }
}

void RoutingGraphBuilder::addSidewayEdge(LaneChangeLaneletsCollector& laneChangeLanelets, const ConstLanelet& ll,
                                         const ConstLineString3d& bound, const RelationType& relation) {
  auto directlySideway = [&relation, &ll](const ConstLanelet& sideLl) {
    return relation == RelationType::AdjacentLeft ? geometry::leftOf(sideLl, ll) : geometry::rightOf(sideLl, ll);
  };
  auto sideOf = pointsToLanelets_.equal_range(orderedIdPair(bound.front().id(), bound.back().id()));
  for (auto it = sideOf.first; it != sideOf.second; ++it) {
    if (ll != it->second && !hasEdge(ll, it->second) && directlySideway(it->second)) {
      if (trafficRules_.canChangeLane(ll, it->second)) {
        // we process lane changes later, when we know all lanelets that can participate in lane change
        laneChangeLanelets.add(ll, it->second);
      } else {
        assignCosts(ll, it->second, relation);
      }
    }
  }
}

void RoutingGraphBuilder::addConflictingEdge(const ConstLanelet& ll, const LaneletLayer& passableLanelets) {
  // Conflicting
  ConstLanelets results = passableLanelets.search(geometry::boundingBox2d(ll));
  ConstLanelet other;
  for (auto& result : results) {
    if (bothWaysLaneletIds_.find(ll.id()) != bothWaysLaneletIds_.end() && result == ll) {
      other = result.invert();
      assignCosts(ll, other, RelationType::Conflicting);
      assignCosts(other, ll, RelationType::Conflicting);
      continue;
    }
    other = result;
    if (hasEdge(ll, result)) {
      continue;
    }
    auto vertex = graph_->getVertex(other);
    if (!vertex || result == ll) {
      continue;
    }
    auto maxHeight = participantHeight();
    if ((maxHeight && geometry::overlaps3d(ll, other, *maxHeight)) || (!maxHeight && geometry::overlaps2d(ll, other))) {
      assignCosts(ll, other, RelationType::Conflicting);
      assignCosts(other, ll, RelationType::Conflicting);
    }
  }
}

void RoutingGraphBuilder::addLaneChangeEdges(LaneChangeLaneletsCollector& laneChanges, const RelationType& relation) {
  auto getSuccessors = [this](auto beginEdgeIt, auto endEdgeIt, auto getVertex) {
    ConstLanelets nexts;
    for (; beginEdgeIt != endEdgeIt; ++beginEdgeIt) {
      auto& edgeInfo = graph_->get()[*beginEdgeIt];
      if (edgeInfo.relation == RelationType::Successor && edgeInfo.costId == 0) {
        nexts.push_back(graph_->get()[getVertex(*beginEdgeIt, graph_->get())].lanelet());
      }
    }
    return nexts;
  };
  auto next = [this, &getSuccessors](const ConstLanelet& llt) {
    auto edges = boost::out_edges(*graph_->getVertex(llt), graph_->get());
    return getSuccessors(edges.first, edges.second, [](auto edge, const auto& g) { return boost::target(edge, g); });
  };
  auto prev = [this, &getSuccessors](const ConstLanelet& llt) {
    auto edges = boost::in_edges(*graph_->getVertex(llt), graph_->get());
    return getSuccessors(edges.first, edges.second, [](auto edge, const auto& g) { return boost::source(edge, g); });
  };
  Optional<LaneChangeLaneletsCollector::LaneChangeLanelets> laneChangeLanelets;
  while (!!(laneChangeLanelets = laneChanges.getNextChangeLanelets(prev, next))) {
    assignLaneChangeCosts(std::move(laneChangeLanelets->first), std::move(laneChangeLanelets->second), relation);
  }
}

void RoutingGraphBuilder::addAreaEdge(const ConstArea& area, const LaneletLayer& passableLanelets) {
  auto candidates = passableLanelets.search(geometry::boundingBox2d(area));
  for (auto& candidate : candidates) {
    bool canPass = false;
    if (trafficRules_.canPass(area, candidate)) {
      canPass = true;
      assignCosts(area, candidate, RelationType::Area);
    }
    if (trafficRules_.canPass(area, candidate.invert())) {
      canPass = true;
      assignCosts(area, candidate.invert(), RelationType::Area);
    }
    if (trafficRules_.canPass(candidate, area)) {
      canPass = true;
      assignCosts(candidate, area, RelationType::Area);
    }
    if (trafficRules_.canPass(candidate.invert(), area)) {
      canPass = true;
      assignCosts(candidate.invert(), area, RelationType::Area);
    }
    if (canPass) {
      continue;
    }
    auto maxHeight = participantHeight();
    if ((maxHeight && geometry::overlaps3d(area, candidate, *maxHeight)) ||
        (!maxHeight && geometry::overlaps2d(area, candidate))) {
      assignCosts(candidate, area, RelationType::Conflicting);
    }
  }
}

void RoutingGraphBuilder::addAreaEdge(const ConstArea& area, const AreaLayer& passableAreas) {
  auto candidates = passableAreas.search(geometry::boundingBox2d(area));
  for (auto& candidate : candidates) {
    if (candidate == area) {
      continue;
    }
    if (trafficRules_.canPass(area, candidate)) {
      assignCosts(area, candidate, RelationType::Area);
      continue;
    }
    auto maxHeight = participantHeight();
    if ((maxHeight && geometry::overlaps3d(ConstArea(area), candidate, *maxHeight)) ||
        (!maxHeight && geometry::overlaps2d(ConstArea(area), candidate))) {
      assignCosts(candidate, area, RelationType::Conflicting);
    }
  }
}

Optional<double> RoutingGraphBuilder::participantHeight() const {
  auto height = config_.find(RoutingGraph::ParticipantHeight);
  if (height != config_.end()) {
    return height->second.asDouble();
  }
  return {};
}

void RoutingGraphBuilder::addPointsToSearchIndex(const ConstLanelet& ll) {
  using PointLaneletPair = std::pair<IdPair, ConstLanelet>;
  pointsToLanelets_.insert(
      PointLaneletPair(orderedIdPair(ll.leftBound().front().id(), ll.rightBound().front().id()), ll));
  pointsToLanelets_.insert(
      PointLaneletPair(orderedIdPair(ll.leftBound().back().id(), ll.rightBound().back().id()), ll));
  pointsToLanelets_.insert(
      PointLaneletPair(orderedIdPair(ll.leftBound().front().id(), ll.leftBound().back().id()), ll));
  pointsToLanelets_.insert(
      PointLaneletPair(orderedIdPair(ll.rightBound().front().id(), ll.rightBound().back().id()), ll));
}

bool RoutingGraphBuilder::hasEdge(const ConstLanelet& from, const ConstLanelet& to) {
  return !!graph_->getEdgeInfo(from, to);
}

void RoutingGraphBuilder::assignLaneChangeCosts(ConstLanelets froms, ConstLanelets tos, const RelationType& relation) {
  assert(relation == RelationType::Left || relation == RelationType::Right);
  assert(froms.size() == tos.size());
  for (; !froms.empty(); froms.erase(froms.begin()), tos.erase(tos.begin())) {
    for (RoutingCostId costId = 0; costId < RoutingCostId(routingCosts_.size()); ++costId) {
      auto cost = routingCosts_[costId]->getCostLaneChange(trafficRules_, froms, tos);
      if (!std::isfinite(cost)) {
        // if the costs are infinite, we add an adjacent edge instead
        auto adjacent = relation == RelationType::Left ? RelationType::AdjacentLeft : RelationType::AdjacentRight;
        graph_->addEdge(froms.front(), tos.front(), EdgeInfo{1, costId, adjacent});
        continue;
      }
      graph_->addEdge(froms.front(), tos.front(), EdgeInfo{cost, costId, relation});
    }
  }
}

void RoutingGraphBuilder::assignCosts(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to,
                                      const RelationType& relation) {
  for (RoutingCostId rci = 0; rci < RoutingCostId(routingCosts_.size()); rci++) {
    EdgeInfo edgeInfo{};
    edgeInfo.costId = rci;
    edgeInfo.relation = relation;
    auto& routingCost = *routingCosts_[rci];
    if (relation == RelationType::Successor || relation == RelationType::Area) {
      edgeInfo.routingCost = routingCost.getCostSucceeding(trafficRules_, from, to);
    } else if (relation == RelationType::Left || relation == RelationType::Right) {
      edgeInfo.routingCost = routingCost.getCostLaneChange(trafficRules_, {*from.lanelet()}, {*to.lanelet()});
    } else if (relation == RelationType::AdjacentLeft || relation == RelationType::AdjacentRight ||
               relation == RelationType::Conflicting) {
      edgeInfo.routingCost = 1;
    } else {
      assert(false && "Trying to add edge with wrong relation type to graph.");  // NOLINT
      return;
    }
    graph_->addEdge(from, to, edgeInfo);
  }
}
}  // namespace internal
}  // namespace routing
}  // namespace lanelet
