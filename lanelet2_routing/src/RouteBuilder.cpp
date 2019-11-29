#include "internal/RouteBuilder.h"
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <unordered_set>
#include "Graph.h"
#include "internal/GraphUtils.h"

namespace lanelet {
namespace routing {
namespace {
struct VisitationCount {
  bool isLeaf() const { return numFollowers + numLaneChangesOut == 0; }
  unsigned numFollowers{};
  unsigned numLaneChangesOut{};
};

//! Explicit representation into which the implicit filtered_graph from boost is parsed. Contains functionality to add
//! or delete vertices.
class VisitedLaneletGraph {
  using VisitedCounters = std::map<NextToRouteGraph::vertex_descriptor, Optional<VisitationCount>>;

 public:
  using VisitedVertex = VisitedCounters::value_type;

  explicit VisitedLaneletGraph(const RouteLanelets& llts) : routeLanelets_{&llts} {}

  //! To be called on each new vertex once it is discovered
  void init(LaneletVertexId v) {
    assert(!visited_[v]);
    visited_[v] = VisitationCount{};
  }

  //! Adds an edge
  void add(LaneletVertexId v, NextToRouteGraph::edge_descriptor e, const NextToRouteGraph& g) {
    auto type = g[e].relation;
    assert(!!visited_[v] && "init() should make sure vertex is valid!");
    if (type == RelationType::Left || type == RelationType::Right) {
      auto dest = boost::target(e, g);
      if (routeLanelets_->find(dest) != routeLanelets_->end()) {
        visited_[v]->numLaneChangesOut++;
      }
    } else if (type == RelationType::Successor) {
      visited_[v]->numFollowers++;
    } else {
      assert(false && "This is not possible if the edge filter did his job!");
    }
  }

  //! Prunes all leafs if this graph except for the lanelet llt
  void pruneLeafsExcluding(LaneletVertexId llt, const NextToRouteGraph& g) {
    bool progress = true;
    while (progress) {
      progress = false;
      for (auto& vertex : visited_) {
        if (vertex.first == llt || !vertex.second || !vertex.second->isLeaf()) {
          continue;
        }
        progress = true;
        remove(vertex, g);
      }
    }
  }

  //! Clears the graph
  void clear() {
    for (auto& vertex : visited_) {
      vertex.second.reset();
    }
  }

  //! Iterates the vertices of the graph
  template <typename Func>
  void forAllValidLanelets(Func&& f) const {
    for (auto& vertex : visited_) {
      if (!!vertex.second && !vertex.second->isLeaf()) {
        f(vertex);
      }
    }
  }

  //! Remove a vertex from the graph
  void remove(LaneletVertexId v, const NextToRouteGraph& g) {
    auto iter = visited_.find(v);
    assert(iter != visited_.end());
    remove(*iter, g);
  }

 private:
  void remove(VisitedCounters::value_type& v, const NextToRouteGraph& g) {
    v.second.reset();
    auto inEdges = boost::in_edges(v.first, g);
    std::for_each(inEdges.first, inEdges.second, [&](NextToRouteGraph::edge_descriptor e) {
      auto relation = g[e].relation;
      if ((relation & (RelationType::Successor | RelationType::Left | RelationType::Right)) == RelationType::None) {
        return;
      }
      auto vertex = visited_.find(boost::source(e, g));
      if (vertex == visited_.end() || !vertex->second) {
        return;
      }
      if (relation == RelationType::Successor && vertex->second->numFollowers > 0) {
        vertex->second->numFollowers--;
      } else if (vertex->second->numLaneChangesOut > 0) {  // implies left or right
        vertex->second->numLaneChangesOut--;
      }
    });
  }
  VisitedCounters visited_;
  const RouteLanelets* routeLanelets_{};
};

//! The visited lanelet graph initially also contains lanelets that are always conflicting with the route, but the route
//! is not always conflicting with them. This class finds these cases.
class PathsOutOfRouteFinder {
 public:
  PathsOutOfRouteFinder(const OriginalGraph& g, const RouteLanelets& llts)
      : g_{g, OnlyDrivableEdgesFilter{g}},
        gRoute_{g, {}, OnRouteFilter{llts}},
        gNoConf_{g, NoConflictingFilter{g}},
        conflictsWithRoute_{g, llts},
        iterRoute_{ConflictOrAdjacentToRouteGraph{g, OnlyDrivableEdgesFilter{g}, conflictsWithRoute_}},
        iterPath_{ConflictsWithPathGraph{g, NoConflictingFilter{g},
                                         OnRouteAndConflictFilter{llts, newConflictingVertices_, g}}},
        llts_{&llts} {}

  //! Iterates vertices that are currently within the graph but the route is not always connected to them. The approach
  //! to finding them is to find sequences of lanelets that would be added to the route in which every lanelet is not
  //! connected with the remaining route through lane changes. For these sequences we search a path in the current route
  //! that connects the first and the last lanelet of the sequence. Every lanelet in the path must be in conflict with
  //! the sequence. If such a path exists, we keep the lanelets.
  template <typename Func>
  void forEachVertexOutOfRoute(const VisitedLaneletGraph& visited, Func&& f) {
    newConflictingVertices_.clear();
    permittedVertices_.clear();
    visited.forAllValidLanelets([&](const VisitedLaneletGraph::VisitedVertex& v) {
      if (conflictsWithRoute_(v.first)) {
        newConflictingVertices_.push_back(v.first);
      }
    });
    permittedVertices_.resize(newConflictingVertices_.size(), false);
    auto testIfPathIsPermitted = [&](auto& path) {
      // neverConflictsWith is a shortcut for cases where all lanelets are only adjacent but not conflicting.
      if (neverConflictsWithRoute(path) || alwaysConflictsWithRoute(path)) {
        for (auto& vertex : path) {
          auto iter = std::find(newConflictingVertices_.begin(), newConflictingVertices_.end(), vertex);
          permittedVertices_[std::distance(newConflictingVertices_.begin(), iter)] = true;
        }
      }
    };
    for (auto newVertex : newConflictingVertices_) {
      auto inEdges = boost::in_edges(newVertex, g_);
      auto connectsToRoute = [&](auto e) {
        constexpr auto AllowedRelation = RelationType::Successor | RelationType::Left | RelationType::Right;
        return hasRelation<AllowedRelation>(g_, e) && has(*llts_, boost::source(e, g_));
      };
      if (std::any_of(inEdges.first, inEdges.second, connectsToRoute)) {
        iterRoute_.forEachPath(newVertex, testIfPathIsPermitted);
      }
    }
    for (auto permit = permittedVertices_.begin(); permit != permittedVertices_.end(); ++permit) {
      if (!*permit) {
        f(newConflictingVertices_[size_t(std::distance(permittedVertices_.begin(), permit))]);
      }
    }
  }

 private:
  bool neverConflictsWithRoute(const LaneletVertexIds& path) {
    auto& g = gNoConf_;
    auto isAdjacentToRoute = [&](LaneletVertexId v) {
      auto inEdges = boost::in_edges(v, g);
      auto outEdges = boost::out_edges(v, g);
      constexpr auto Adjacent =
          RelationType::Left | RelationType::Right | RelationType::AdjacentLeft | RelationType::AdjacentRight;
      return std::any_of(inEdges.first, inEdges.second,
                         [&](auto e) { return hasRelation<Adjacent>(g, e) && has(*llts_, boost::source(e, g)); }) ||
             std::any_of(outEdges.first, outEdges.second,
                         [&](auto e) { return hasRelation<Adjacent>(g, e) && has(*llts_, boost::target(e, g)); });
    };
    return std::all_of(path.begin(), path.end(), isAdjacentToRoute);
  }
  bool alwaysConflictsWithRoute(const LaneletVertexIds& path) {
    // We have to find a path within the route that is always conflicting with the new path
    assert(!path.empty());
    auto before = boost::in_edges(path.front(), g_);
    auto after = boost::out_edges(path.back(), g_);
    auto fromE = std::find_if(before.first, before.second, [&](auto e) { return has(*llts_, boost::source(e, g_)); });
    auto toE = std::find_if(after.first, after.second, [&](auto e) { return has(*llts_, boost::target(e, g_)); });
    if (fromE == before.second || toE == after.second) {
      return false;  // path is not connected to current route
    }
    auto from = boost::source(*fromE, g_);
    auto to = boost::target(*toE, g_);
    iterPath_.graph().m_vertex_pred.setConflicting(path);
    iterPath_.graph().m_vertex_pred.setStart(from);
    iterPath_.graph().m_vertex_pred.setEnd(to);

    return iterPath_.hasPathFromTo(from, to);
  }
  DrivableGraph g_;
  RouteGraph gRoute_;
  NoConflictingGraph gNoConf_;
  std::vector<LaneletVertexId> newConflictingVertices_;
  std::vector<bool> permittedVertices_;
  ConflictingSectionFilter conflictsWithRoute_;
  ConnectedPathIterator<ConflictOrAdjacentToRouteGraph> iterRoute_;
  ConnectedPathIterator<ConflictsWithPathGraph> iterPath_;
  const RouteLanelets* llts_{};
};

//! Adaptor for boost graph. It is used to build the VisitedLaneletGraph.
class NeighbouringGraphVisitor : public boost::default_bfs_visitor {
 public:
  explicit NeighbouringGraphVisitor(VisitedLaneletGraph& counter) : counter_{&counter} {}
  // called by boost graph on a new vertex
  void examine_vertex(LaneletVertexId v, const NextToRouteGraph& /*g*/) {  // NOLINT
    counter_->init(v);
    vCurr_ = v;
  }

  // called directly after an all its edges
  void examine_edge(NextToRouteGraph::edge_descriptor e, const NextToRouteGraph& g) const {  // NOLINT
    assert(vCurr_ == boost::source(e, g) && "Breadth first search seems to iterate in a weird manner");
    counter_->add(vCurr_, e, g);
  }

 private:
  LaneletVertexId vCurr_{};
  VisitedLaneletGraph* counter_;
};

//! A visitor that constructs the final route object by searching though the graph of lanelets on the route.
class RouteConstructionVisitor : public boost::default_bfs_visitor {
 public:
  RouteConstructionVisitor() = default;
  RouteConstructionVisitor(const Graph& graph, ConstLaneletRouteElementMap& elements, LaneBegins& laneBegins)
      : graph_{&graph}, elements_{&elements}, laneBegins_{&laneBegins} {}

  //! called by boost graph on a new vertex
  void examine_vertex(LaneletVertexId v, const RouteGraph& g) {  // NOLINT
    auto llt = g[v].lanelet();
    if (elements_->empty()) {
      // first time
      sourceElem_ = elements_->emplace(llt, std::make_unique<RouteElement>(llt, laneId_)).first;
      laneBegins_->emplace(laneId_, sourceElem_->second.get());
    } else {
      sourceElem_ = elements_->find(llt);
    }
  }

  //! called directly after examine_vertex on all its edges
  void examine_edge(RouteGraph::edge_descriptor e, const RouteGraph& g) {  // NOLINT
    auto dest = boost::target(e, g);
    auto llt = g[dest].lanelet();
    auto destElem = elements_->find(llt);
    const auto newLane = isDifferentLane(e, g);
    auto type = g[e].relation;
    if (destElem == elements_->end()) {
      const auto laneId = newLane ? ++laneId_ : sourceElem_->second->laneId();
      destElem = elements_->emplace(llt, std::make_unique<RouteElement>(llt, laneId)).first;
      if (newLane) {
        laneBegins_->emplace(laneId, destElem->second.get());
      }
    } else if (destElem != elements_->end() && !newLane && type == RelationType::Successor &&
               sourceElem_->second->laneId() != destElem->second->laneId()) {
      // this happens if we reached a lanelet because it was part of a circle of reached through a conflicting lanelet.
      // In this case we have to adjust lane Ids
      setLaneIdFromTo(destElem->second->laneId(), sourceElem_->second->laneId());
    }
    auto* destRelation = destElem->second.get();
    switch (type) {
      case RelationType::Left:
      case RelationType::AdjacentLeft:
        sourceElem_->second->setLeft(RouteElementRelation{destRelation, type});
        break;
      case RelationType::Right:
      case RelationType::AdjacentRight:
        sourceElem_->second->setRight(RouteElementRelation{destRelation, type});
        break;
      case RelationType::Successor:
        sourceElem_->second->addFollowing(destRelation);
        destRelation->addPrevious(sourceElem_->second.get());
        break;
      case RelationType::Conflicting:
        sourceElem_->second->addConflictingInRoute(destRelation);
        sourceElem_->second->addConflictingInMap(destRelation->lanelet());
        break;
      default:
        break;
    }
  }
  void finish_vertex(LaneletVertexId v, const RouteGraph& g) {  // NOLINT
    // now that all neighbours are known, now is the time to add lanelets that conflict in the whole graph
    auto& routingGraph = graph_->get();
    auto numOut = boost::out_degree(v, g);
    auto numOutGraph = boost::out_degree(v, routingGraph);
    assert(numOutGraph >= numOut &&
           "the original routing graph must not have more less edges than the graph of the route!");
    if (numOut == numOutGraph) {
      // there cant be any conflicting for this lanelet, all edges lie within the graph of the route. Only works if
      // there is only one routing cost object.
      return;
    }
    auto outGraph = boost::out_edges(v, routingGraph);
    std::for_each(outGraph.first, outGraph.second, [&](GraphTraits::edge_descriptor e) {
      if (routingGraph[e].relation != RelationType::Conflicting) {
        return;
      }
      auto dest = boost::target(e, g);
      auto destLlt = g[dest];
      if (destLlt.laneletOrArea.isLanelet() && elements_->find(destLlt.lanelet()) != elements_->end()) {
        return;  // this lanelet is part of the route
      }
      sourceElem_->second->addConflictingInMap(destLlt.laneletOrArea);
    });
  }

 private:
  void setLaneIdFromTo(RouteElement::LaneId from, RouteElement::LaneId to) {
    for (auto& elem : *elements_) {
      if (elem.second->laneId() == from) {
        elem.second->setLaneId(to);
      }
    }
    auto laneBegin = laneBegins_->find(from);
    assert(laneBegin != laneBegins_->end());
    laneBegins_->erase(laneBegin);
  }

  bool isDifferentLane(RouteGraph::edge_descriptor e, const RouteGraph& g) const {
    // we increment the lane id whenever the vertex has more than one predecessor or the singele predecessor has
    // multiple follower. Non-Successor edges are always a different lane.
    if (g[e].relation != RelationType::Successor) {
      return true;
    }
    auto dest = boost::target(e, g);
    if (!hasOneSuccessor(boost::in_edges(dest, g), g)) {
      return true;
    }
    auto source = boost::source(e, g);
    return !hasOneSuccessor(boost::out_edges(source, g), g);
  }

  static LaneletVertexId getOnePredecessor(LaneletVertexId v, const RouteGraph& g) {
    auto inEdges = boost::in_edges(v, g);
    auto e =
        *std::find_if(inEdges.first, inEdges.second, [&](auto e) { return g[e].relation == RelationType::Successor; });
    return boost::source(e, g);
  }

  template <typename EdgesT>
  static bool hasOneSuccessor(const EdgesT& edges, const RouteGraph& g) {
    bool hasOneEdge = false;
    for (auto it = edges.first; it != edges.second; ++it) {
      bool oneMoreEdge = g[*it].relation == RelationType::Successor;
      if (oneMoreEdge && hasOneEdge) {
        return false;
      }
      hasOneEdge |= oneMoreEdge;
    }
    return hasOneEdge;
  }

  const Graph* graph_{};
  RouteElement::LaneId laneId_{1000};
  ConstLaneletRouteElementMap* elements_{};
  ConstLaneletRouteElementMap::iterator sourceElem_;
  LaneBegins* laneBegins_{};
};

template <typename Graph, typename StartVertex, typename Visitor>
void breadthFirstSearch(const Graph& g, StartVertex v, Visitor vis) {
  SparseColorMap cm;
  boost::queue<StartVertex> q;
  boost::breadth_first_visit(g, v, q, vis, cm);
}

//! This is the class that handles building the route. It iteratively adds lanelets initial route (i.e. the shortest
//! path) that fulfill the definition of a lanelet on the route. This is done until convergence is reached.
class RouteUnderConstruction {
 public:
  RouteUnderConstruction(const std::vector<LaneletVertexId>& initialRoute, const OriginalGraph& originalGraph)
      : begin_{initialRoute.front()},
        end_{initialRoute.back()},
        laneletsOnRoute_{initialRoute.begin(), initialRoute.end()},
        originalGraph_{originalGraph},
        routeGraph_{originalGraph_, {}, OnRouteFilter{laneletsOnRoute_}},
        nextToRouteGraph_{originalGraph_, OnlyDrivableEdgesFilter{originalGraph_},
                          NextToRouteFilter{laneletsOnRoute_, originalGraph_}},
        laneletVisitor_{laneletsOnRoute_},
        pathOutOfRouteFinder_{originalGraph_, laneletsOnRoute_} {}

  //! Adds neighbours of the current route to the route if they fulfill the criteria. Returns true if something was
  //! added.
  bool addAdjacentLaneletsToRoute() {
    laneletVisitor_.clear();

    // query the (implicit) boost graph of adjacent lanelets into the nextToRouteGraph_
    breadthFirstSearch(nextToRouteGraph_, begin_, NeighbouringGraphVisitor{laneletVisitor_});

    // remove lanelets that do not lead to end_ from the graph
    laneletVisitor_.pruneLeafsExcluding(end_, nextToRouteGraph_);

    // remove lanelets that (temporarily) leave the route from the graph
    pathOutOfRouteFinder_.forEachVertexOutOfRoute(
        laneletVisitor_, [&](auto& vertex) { laneletVisitor_.remove(vertex, nextToRouteGraph_); });

    // add the new vertices to the laneletsOnTheRoute_ and determine if we made progress.
    bool progress = false;
    laneletVisitor_.forAllValidLanelets([&](auto lltId) { progress |= laneletsOnRoute_.insert(lltId.first).second; });
    return progress;
  }

  //! Creates the route object from the current state of the route.
  Optional<Route> finalizeRoute(const Graph& theGraph, const LaneletPath& thePath) {
    ConstLaneletRouteElementMap elements;
    LaneBegins laneBegins;
    // Search the graph of the route. The visitor fills elements and lanebegins
    RouteConstructionVisitor visitor(theGraph, elements, laneBegins);
    breadthFirstSearch(routeGraph_, begin_, visitor);

    if (elements.find(thePath.back()) == elements.end()) {
      return {};  // there was no path between begin and end
    }

    auto map = utils::createConstMap(utils::transform(elements, [](auto& elem) { return elem.first; }), {});
    return Route(thePath, std::move(elements), std::move(laneBegins), std::move(map));
  }

 private:
  LaneletVertexId begin_;
  LaneletVertexId end_;
  RouteLanelets laneletsOnRoute_;  //! Lanelets already determined to be on the route
  const OriginalGraph& originalGraph_;
  RouteGraph routeGraph_;
  NextToRouteGraph nextToRouteGraph_;
  VisitedLaneletGraph laneletVisitor_;
  PathsOutOfRouteFinder pathOutOfRouteFinder_;
};
}  // namespace

//! This is the algorithm in its most simple form. For efficiency reasons, all internal operations are not based on the
//! lanelets but on the vertex ids that boost::graph has assigned to them.
Optional<Route> RouteBuilder::getRouteFromShortestPath(const LaneletPath& path, bool withLaneChanges,
                                                       RoutingCostId costId) {
  // translate the lanelets to graph ids
  auto vertexIds = utils::transform(path, [&](auto llt) { return LaneletVertexId{*graph_.getVertex(llt)}; });
  // decide which graph we are going to use
  OriginalGraph originalGraph{graph_.get(), OriginalGraphFilter{graph_.get(), withLaneChanges, costId}};

  // get the container for all the things
  RouteUnderConstruction routeUnderConstruction{vertexIds, originalGraph};

  //! @todo deleteme
  auto ids = utils::transform(graph_.get().m_vertices, [&](auto& v) { return v.m_property.laneletOrArea.id(); });

  bool progress = true;
  while (progress) {
    progress = routeUnderConstruction.addAdjacentLaneletsToRoute();
  }
  return routeUnderConstruction.finalizeRoute(graph_, path);
}
}  // namespace routing
}  // namespace lanelet
