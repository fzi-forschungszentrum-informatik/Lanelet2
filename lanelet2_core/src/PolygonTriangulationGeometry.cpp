#include <boost/polygon/voronoi.hpp>

#include "lanelet2_core/geometry/Polygon.h"

using VoronoiPrecision = std::int64_t;

namespace {
VoronoiPrecision toVoronoi(double v) { return static_cast<VoronoiPrecision>(v * 100); }
}  // namespace

using AlignedBasicPoint2d = Eigen::Matrix<double, 2, 1>;

namespace boost {
namespace polygon {
template <>
struct geometry_concept<lanelet::ConstPoint2d> {
  using type = point_concept;  // NOLINT
};

// specialize getter
template <>
struct point_traits<lanelet::ConstPoint2d> {
  using coordinate_type = VoronoiPrecision;  // NOLINT
  static inline coordinate_type get(const lanelet::ConstPoint2d& point, const orientation_2d& orient) {
    return toVoronoi((orient == HORIZONTAL) ? point.x() : point.y());
  }
};

// specialize setter
template <>
struct point_mutable_traits<lanelet::ConstPoint2d> {
  using coordinate_type = VoronoiPrecision;  // NOLINT

  static inline void set(lanelet::ConstPoint2d& /*point*/, const orientation_2d& /*orient*/,
                         VoronoiPrecision /*value*/) {
    assert(false && "boost polygon should never call this!");
  }
};

template <>
struct geometry_concept<lanelet::BasicPoint2d> {
  using type = point_concept;  // NOLINT
};

// specialize getter
template <>
struct point_traits<lanelet::BasicPoint2d> {
  using coordinate_type = VoronoiPrecision;  // NOLINT
  static inline coordinate_type get(const lanelet::BasicPoint2d& point, const orientation_2d& orient) {
    return toVoronoi((orient == HORIZONTAL) ? point.x() : point.y());
  }
};

// specialize setter
template <>
struct point_mutable_traits<lanelet::BasicPoint2d> {
  using coordinate_type = VoronoiPrecision;  // NOLINT

  static inline void set(lanelet::BasicPoint2d& /*point*/, const orientation_2d& /*orient*/,
                         VoronoiPrecision /*value*/) {
    assert(false && "boost polygon should never call this!");
  }
};

template <>
struct geometry_concept<AlignedBasicPoint2d> {
  using type = point_concept;  // NOLINT
};

// specialize getter
template <>
struct point_traits<AlignedBasicPoint2d> {
  using coordinate_type = VoronoiPrecision;  // NOLINT
  static inline coordinate_type get(const AlignedBasicPoint2d& point, const orientation_2d& orient) {
    return toVoronoi((orient == HORIZONTAL) ? point.x() : point.y());
  }
};

// specialize setter
template <>
struct point_mutable_traits<AlignedBasicPoint2d> {
  using coordinate_type = VoronoiPrecision;  // NOLINT

  static inline void set(AlignedBasicPoint2d& /*point*/, const orientation_2d& /*orient*/, VoronoiPrecision /*value*/) {
    assert(false && "boost polygon should never call this!");
  }
};
}  // namespace polygon
}  // namespace boost

namespace {
using namespace lanelet;
using namespace lanelet::geometry;

using IndexedPolygon = std::vector<size_t>;
using IndexedPolygons = std::vector<IndexedPolygon>;

bool isConnectionConvex(const BasicPoint2d& seg1, const BasicPoint2d& seg2,
                        const double eps = 4 * std::numeric_limits<double>::epsilon()) {
  return lanelet::geometry::internal::crossProd(seg1.normalized(), seg2.normalized()).z() <= eps;
}
using IndexedSegment = std::pair<size_t, size_t>;
using IndexedSegments = std::vector<IndexedSegment>;
using IndexedSegmentPair = std::pair<IndexedSegment, IndexedSegment>;
IndexedSegment flip(const IndexedSegment& seg) { return std::make_pair(seg.second, seg.first); }
bool isBorder(const IndexedSegment& seg, const size_t polySize) {
  const auto distance = seg.first > seg.second ? seg.first - seg.second : seg.second - seg.first;
  if (distance == 1) {
    return true;
  }
  return (seg.first == 0 || seg.second == 0) && distance + 1 == polySize;
}

using PolygonIterator = IndexedPolygon::const_iterator;
using PolygonIteratorPair = std::pair<PolygonIterator, PolygonIterator>;
using VoronoiGraph = boost::polygon::voronoi_diagram<double>;

PolygonIteratorPair getAlignedIterators(const IndexedPolygon& poly, const IndexedSegment& segment) {
  if (poly.size() < 3) {
    throw InvalidInputError("Can't find segment from polygon with less than 3 vertices");
  }
  PolygonIteratorPair itPair = std::make_pair(std::find(poly.begin(), poly.end(), segment.first),
                                              std::find(poly.begin(), poly.end(), segment.second));
  if (itPair.first == poly.end() || itPair.second == poly.end()) {
    throw InvalidInputError("Index " + std::to_string(segment.first) + "-" + std::to_string(segment.second) +
                            " not found in indices");
  }
  const auto dist = std::distance(itPair.first, itPair.second);
  if (dist == -1 || dist == poly.size() - 1) {
    std::swap(itPair.first, itPair.second);
  }
  return itPair;
}

IndexedSegmentPair getAdjacent(const IndexedPolygon& poly, const IndexedSegment& seg) {
  auto itP = getAlignedIterators(poly, seg);
  return {{*itP.first, itP.first == poly.begin() ? poly.back() : *std::prev(itP.first)},
          {*itP.second, itP.second == std::prev(poly.end()) ? poly.front() : *std::next(itP.second)}};
}

IndexedSegments getSegmentsExcept(const IndexedPolygon& p, const IndexedSegment& excluded) {
  IndexedSegments res;
  res.reserve(p.size() - 1);
  for (size_t i = 0; i < p.size(); ++i) {
    IndexedSegment seg{p.at(i), p.at((i + 1) % p.size())};
    if (seg != excluded && seg != flip(excluded)) {
      res.emplace_back(seg);
    }
  }
  return res;
}

IndexedSegments getSegmentIndexesExcept(const IndexedPolygon& p, const IndexedSegment& excluded) {
  IndexedSegments res;
  res.reserve(p.size() - 1);
  for (size_t i = 0; i < p.size(); ++i) {
    IndexedSegment seg{p.at(i), p.at((i + 1) % p.size())};
    if (seg != excluded && seg != flip(excluded)) {
      res.emplace_back(IndexedSegment{i, (i + 1) % p.size()});
    }
  }
  return res;
}

IndexedPolygon removeSegment(const IndexedPolygon& poly, const IndexedSegment& excluded) {
  auto itP = getAlignedIterators(poly, excluded);
  IndexedPolygon res(std::next(itP.second), std::next(itP.first) == poly.end() ? itP.first : poly.end());
  if (itP.first != std::prev(poly.end())) {
    res.insert(res.end(), poly.begin(), itP.first);
  }
  return res;
}

struct MergeResult {
  IndexedPolygon merged;
  IndexedSegments addedSegments;
};

void insertPolygonExcept(IndexedPolygon& poly, const IndexedPolygon& toMerge, const IndexedSegment& border) {
  auto mergeIt = getAlignedIterators(toMerge, border);
  auto npIt = getAlignedIterators(poly, border);
  auto noWrap = std::next(mergeIt.first) == toMerge.end();
  poly.insert(npIt.second, std::next(mergeIt.second), noWrap ? mergeIt.first : toMerge.end());
  if (!noWrap) {
    poly.insert(std::next(npIt.second, std::distance(std::next(mergeIt.second), toMerge.end())), toMerge.begin(),
                mergeIt.first);
  }
}

MergeResult merge(const IndexedPolygon& poly, const IndexedPolygon& toMerge, const IndexedSegment& border) {
  MergeResult result{{}, getSegmentsExcept(toMerge, border)};
  result.merged.reserve(poly.size() + toMerge.size());  // das hier ist die einzige Allokation
  result.merged = poly;
  insertPolygonExcept(result.merged, toMerge, border);
  return result;
}

using MetricSegmentPair = std::pair<BasicPoint2d, BasicPoint2d>;

MetricSegmentPair getMetricAdjacentSegmentPair(const BasicPolygon2d& metricPolygon, const IndexedPolygon& poly,
                                               const IndexedSegment& segment) {
  const auto newSegments = getAdjacent(poly, segment);
  const BasicPoint2d newFirst = metricPolygon.at(newSegments.first.second) - metricPolygon.at(newSegments.first.first);
  const BasicPoint2d newSecond =
      metricPolygon.at(newSegments.second.second) - metricPolygon.at(newSegments.second.first);
  return {newFirst, newSecond};
}
double getOpposingAngle(const MetricSegmentPair& segmentPair, const size_t polySize) {
  return polySize != 3 ? -1. : segmentPair.first.normalized().dot(segmentPair.second.normalized());
}
bool connectionConvex(const MetricSegmentPair& local, const MetricSegmentPair& extension) {
  return isConnectionConvex(-local.first, extension.second) && isConnectionConvex(-extension.first, local.second);
}
using AdjacentPolygons = std::map<IndexedSegment, size_t>;
using Visited = std::set<size_t>;
struct Gate {
  IndexedSegment border;
  size_t source;
};
using Gates = std::vector<Gate>;
Gates makeGates(const IndexedSegments& borders, const size_t src) {
  return utils::transform(borders, [&](auto& b) { return Gate{b, src}; });
}
struct Head {
  IndexedPolygon poly;
  Gates gates;  // run ccw
};
using Heads = std::vector<Head>;
struct Connection {
  IndexedSegment border;
  double opposingAngle;
  MetricSegmentPair connectors;
  size_t newPoly;
};
using Connections = std::vector<Connection>;
struct TraversalResult {
  std::vector<IndexedPolygon> convex;
  Visited visited;
};

template <typename K, typename V>
class VectorMap {
 public:
  using KeyType = K;
  using ValueType = V;
  using ElementType = std::pair<K, V>;
  using ContainerType = std::vector<ElementType>;

  void append(const KeyType& key, const ValueType& value) {
    if (frozen_) {
      throw std::runtime_error("Can't add to frozen vector map");
    }
    storage_.emplace_back(ElementType{key, value});
  }
  void append(const ElementType& elem) {
    if (frozen_) {
      throw std::runtime_error("Can't add to frozen vector map");
    }
    storage_.emplace_back(elem);
  }
  void freeze() {
    if (!frozen_) {
      std::sort(storage_.begin(), storage_.end());
      frozen_ = true;
    }
  }

  const ValueType& at(const KeyType& key) const {
    if (!frozen_) {
      throw std::runtime_error("Can't search in unfrozen vector map");
    }
    auto it = std::lower_bound(storage_.begin(), storage_.end(), key,
                               [](auto& value, const KeyType& k) { return value.first < k; });
    if (it == storage_.end() || key != it->first) {
      throw std::runtime_error("Item not found in vector map");
    }
    return it->second;
  }

 private:
  ContainerType storage_;
  bool frozen_{false};
};
using Adjacencies = VectorMap<IndexedSegment, size_t>;

class SegmentationData {
 public:
  explicit SegmentationData(const size_t convexPartsSize) { convexParts_.resize(convexPartsSize); }
  void addConvex(const IndexedPolygon& convexPoly, const int idx) { convexParts_[idx] = convexPoly; }
  void addSegment(const IndexedSegment& segment, const size_t adjacentPolygonIdx) {
    adjacent_.append(flip(segment), adjacentPolygonIdx);
  }
  const Adjacencies& getAdjacencies() const { return adjacent_; }
  const IndexedPolygons& getConvexPolygons() const { return convexParts_; }
  size_t getDestination(const Gate& gate) const { return adjacent_.at(gate.border); }
  const IndexedPolygon& getPolygonByIdx(const size_t idx) const { return convexParts_.at(idx); }
  void freeze() { adjacent_.freeze(); }

 private:
  Adjacencies adjacent_;
  std::vector<IndexedPolygon> convexParts_;
};

Connections findGates(const BasicPolygon2d& metricPolygon, const Gates& candidates, const Visited& overallVisited,
                      const SegmentationData& segData) {
  Connections gates;
  for (const auto& gate : candidates) {
    if (isBorder(gate.border, metricPolygon.size())) {
      continue;
    }
    const auto opposingPolygonIdx = segData.getDestination(gate);
    if (overallVisited.find(opposingPolygonIdx) != overallVisited.end()) {
      continue;
    }
    const auto& opposingPoly = segData.getPolygonByIdx(opposingPolygonIdx);
    auto metricSegPair = getMetricAdjacentSegmentPair(metricPolygon, opposingPoly, gate.border);
    gates.emplace_back(Connection{gate.border, getOpposingAngle(metricSegPair, opposingPoly.size()), metricSegPair,
                                  opposingPolygonIdx});
  }

  std::sort(gates.begin(), gates.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.opposingAngle < rhs.opposingAngle; });
  return gates;
}

void updateState(Head& head, TraversalResult& traversalResult, const IndexedPolygon& newPoly, const Connection& conn) {
  traversalResult.visited.insert(conn.newPoly);
  auto mergeResult = merge(head.poly, newPoly, conn.border);
  head.poly = std::move(mergeResult.merged);
  head.gates = makeGates(mergeResult.addedSegments, conn.newPoly);
}

void traverse(const BasicPolygon2d& metricPolygon, Head& head, TraversalResult& traversalResult,
              const SegmentationData& segData, const bool isOrigin) {
  auto gates = findGates(metricPolygon, head.gates, traversalResult.visited, segData);
  std::vector<Connection> closedGates;
  for (const auto& conn : gates) {
    if (traversalResult.visited.find(conn.newPoly) != traversalResult.visited.end()) {
      continue;
    }
    const auto headsideAdjacent = getMetricAdjacentSegmentPair(metricPolygon, head.poly, conn.border);
    if (connectionConvex(headsideAdjacent, conn.connectors)) {
      updateState(head, traversalResult, segData.getPolygonByIdx(conn.newPoly), conn);
      traverse(metricPolygon, head, traversalResult, segData, false);
    } else {
      closedGates.emplace_back(conn);
    }
  }
  if (isOrigin) {
    traversalResult.convex.emplace_back(head.poly);
  }
  for (const auto& conn : closedGates) {
    if (traversalResult.visited.find(conn.newPoly) != traversalResult.visited.end()) {
      continue;
    }
    const auto& newPoly = segData.getPolygonByIdx(conn.newPoly);
    Head newHead{newPoly, makeGates(getSegmentsExcept(newPoly, conn.border), conn.newPoly)};
    traverse(metricPolygon, newHead, traversalResult, segData, true);
  }
}

std::vector<IndexedPolygon> mergeTriangulation(const BasicPolygon2d& metricPolygon, const SegmentationData& segData) {
  IndexedSegment init{1, 0};
  auto firstPolyIdx = segData.getAdjacencies().at(init);
  const auto& firstPoly = segData.getPolygonByIdx(firstPolyIdx);
  TraversalResult t{{}, Visited{firstPolyIdx}};
  Head initHead{firstPoly, makeGates(getSegmentsExcept(firstPoly, init), firstPolyIdx)};
  traverse(metricPolygon, initHead, t, segData, true);
  return t.convex;
}

BasicPolygon2d extractPolygon(const BasicPolygon2d& poly, const std::vector<size_t>& idxs) {
  BasicPolygon2d res(idxs.size());
  for (size_t i{0}; i < idxs.size(); ++i) {
    res[i] = poly.at(idxs.at(i));
  }
  return res;
}

void subdivide(IndexedTriangles& target, const IndexedPolygon& poly) {
  target.reserve(target.size() + poly.size() - 2);
  for (auto first = std::next(poly.begin()), second = std::next(poly.begin(), 2); second != poly.end();
       ++second, ++first) {
    target.emplace_back(IndexedTriangle{poly.front(), *first, *second});
  }
}

bool isTriangular(const IndexedPolygon& poly) { return poly.size() == 3; }

SegmentationData makeVoronoi(const BasicPolygon2d& poly) {
  VoronoiGraph graph;
  boost::polygon::construct_voronoi(poly.begin(), poly.end(), &graph);
  SegmentationData t(graph.num_vertices());
  auto advance = [](const VoronoiGraph::edge_type*& edge, VoronoiGraph::cell_type::source_index_type& lastIndex,
                    VoronoiGraph::cell_type::source_index_type& curIndex) {
    lastIndex = curIndex;
    edge = edge->rot_prev();
    curIndex = edge->cell()->source_index();
  };
  for (size_t i{0}; i < graph.num_vertices(); ++i) {
    const auto* const firstEdge = graph.vertices().at(i).incident_edge();
    auto startIdx = firstEdge->cell()->source_index();
    const auto* curEdge = firstEdge->rot_prev();
    auto curIdx = curEdge->cell()->source_index();
    auto lastIdx = startIdx;
    IndexedPolygon curPoly{startIdx};
    while (curIdx != startIdx) {
      t.addSegment({lastIdx, curIdx}, i);
      curPoly.emplace_back(curIdx);
      advance(curEdge, lastIdx, curIdx);
    }
    t.addSegment({lastIdx, curIdx}, i);
    t.addConvex(curPoly, i);
  }
  return t;
}

BasicPolygons2d convexPartitionImpl(const BasicPolygon2d& poly) {
  if (poly.size() < 3) {
    throw GeometryError("Can't partition a polygon with less than 3 points");
  }
  BasicPolygons2d res;
  auto segData = makeVoronoi(poly);
  segData.freeze();
  auto indexed = mergeTriangulation(poly, segData);
  for (const auto& i : indexed) {
    res.emplace_back(extractPolygon(poly, i));
  }
  return res;
}

IndexedTriangles triangulateImpl(const BasicPolygon2d& poly) {
  if (poly.size() < 3) {
    throw GeometryError("Can't triangulate a polygon with less than 3 points");
  }
  IndexedTriangles res;
  res.reserve(poly.size() - 2);
  auto segData = makeVoronoi(poly);
  for (const auto& i : segData.getConvexPolygons()) {
    if (isTriangular(i)) {
      res.emplace_back(IndexedTriangle{i.at(0), i.at(1), i.at(2)});
    } else {
      subdivide(res, i);
    }
  }
  return res;
}
}  // namespace

namespace lanelet {
namespace geometry {
namespace internal {
BasicPolygons2d convexPartition(const BasicPolygon2d& poly) { return convexPartitionImpl(poly); }
IndexedTriangles triangulate(const BasicPolygon2d& poly) { return triangulateImpl(poly); }
}  // namespace internal
}  // namespace geometry
}  // namespace lanelet
