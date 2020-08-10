#define LANELET_LAYER_DEFINITION
#include "lanelet2_core/LaneletMap.h"

#include <atomic>
#include <boost/geometry/index/rtree.hpp>
#include <chrono>
#include <random>

#include "lanelet2_core/geometry/Area.h"
#include "lanelet2_core/geometry/BoundingBox.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/geometry/RegulatoryElement.h"

// boost geometry stuff
namespace bgi = boost::geometry::index;

// we need this hack to cleanly remove things from the RTree
// because remove() internally uses equals to find the correct element to
// remove. However, this is not implemented for our primitives.
namespace boost {
namespace geometry {
template <>
bool equals<lanelet::LineString3d, lanelet::LineString3d>(const lanelet::LineString3d& geometry1,
                                                          const lanelet::LineString3d& geometry2) {
  return geometry1 == geometry2;
}
template <>
bool equals<lanelet::Polygon3d, lanelet::Polygon3d>(const lanelet::Polygon3d& geometry1,
                                                    const lanelet::Polygon3d& geometry2) {
  return geometry1 == geometry2;
}
}  // namespace geometry
}  // namespace boost

// need id visitor before hash
namespace lanelet {
namespace {

struct IdVisitor : public RuleParameterVisitor {
  Id getId(const ConstRuleParameter& from) {
    boost::apply_visitor(*this, from);
    return this->id;
  }
  void operator()(const ConstPoint3d& p) override { id = p.id(); }
  void operator()(const ConstLineString3d& ls) override { id = ls.id(); }
  void operator()(const ConstPolygon3d& p) override { id = p.id(); }
  void operator()(const ConstWeakLanelet& ll) override {
    if (ll.expired()) {
      return;
    }
    id = ll.lock().id();
  }
  void operator()(const ConstWeakArea& ar) override {
    if (ar.expired()) {
      return;
    }
    id = ar.lock().id();
  }
  Id id{InvalId};
};

template <typename T>
inline Id getId(const T& obj) {
  return obj.id();
}
template <>
inline Id getId<RegulatoryElementPtr>(const RegulatoryElementPtr& obj) {
  return obj->id();
}

template <typename T>
std::unordered_map<Id, T> toMap(const std::vector<T>& vec) {
  using Map = std::unordered_map<Id, T>;
  auto elems = utils::transform(vec, [](auto& elem) { return typename Map::value_type(getId(elem), elem); });
  return std::unordered_map<Id, T>(std::make_move_iterator(elems.begin()), std::make_move_iterator(elems.end()));
}

LineString3d uglyRemoveConst(const ConstLineString3d& ls) {
  // this is ugly but legal, because every const data object used to be a
  // non-const data object when the LineString was initialized.
  auto data = std::const_pointer_cast<LineStringData>(ls.constData());
  return LineString3d(data, ls.inverted());
}
}  // namespace
}  // namespace lanelet

namespace std {
template <>
struct hash<lanelet::ConstRuleParameter> {
  size_t operator()(const lanelet::ConstRuleParameter& x) const noexcept {
    return std::hash<lanelet::Id>()(lanelet::IdVisitor().getId(x));
  }
};
}  // namespace std

namespace lanelet {
using namespace traits;
namespace {
template <typename RType, typename Pair>
auto getSecond(const std::vector<Pair>& p) {
  std::vector<RType> v;
  v.reserve(p.size());
  std::transform(p.cbegin(), p.cend(), std::back_inserter(v), [](const auto& elem) { return elem.second; });
  return v;
}

template <typename PrimT, typename LayerT>
void assignIdIfInvalid(PrimT&& prim, LayerT& layer) {
  static_assert(std::is_same<typename LayerT::PrimitiveT, std::decay_t<PrimT>>::value,
                "Primitive and layer not identical");
  if (prim.id() == InvalId) {
    prim.setId(layer.uniqueId());
  }
}

struct AssignIdVisitor : public boost::static_visitor<void> {
  explicit AssignIdVisitor(LaneletMap* map) : map_(map) {}
  void operator()(Point3d p) { assignIdIfInvalid(p, map_->pointLayer); }
  void operator()(LineString3d l) { assignIdIfInvalid(l, map_->lineStringLayer); }
  void operator()(Polygon3d p) { assignIdIfInvalid(p, map_->polygonLayer); }
  void operator()(const WeakLanelet& ll) {
    if (ll.expired()) {  // NOLINT
      return;
    }
    assignIdIfInvalid(ll.lock(), map_->laneletLayer);
  }
  void operator()(const WeakArea& ar) {
    if (ar.expired()) {
      return;
    }
    assignIdIfInvalid(ar.lock(), map_->areaLayer);
  }

 private:
  LaneletMap* map_;
};

struct AddVisitor : public lanelet::internal::MutableParameterVisitor {
  explicit AddVisitor(LaneletMap* map) : map_(map) {}

  void operator()(const Point3d& p) override { map_->add(p); }
  void operator()(const LineString3d& l) override { map_->add(l); }
  void operator()(const Polygon3d& p) override { map_->add(p); }
  void operator()(const WeakLanelet& ll) override {
    if (ll.expired()) {  // NOLINT
      return;
    }
    map_->add(ll.lock());
  }
  void operator()(const WeakArea& ar) override {
    if (ar.expired()) {  // NOLINT
      return;
    }
    map_->add(ar.lock());
  }

 private:
  LaneletMap* map_;
};

template <typename T, typename MapT, typename KeyT, typename Func>
std::vector<T> forEachMatchInMultiMap(const MapT& map, const KeyT& key, Func&& f) {
  auto range = map.equal_range(key);
  return utils::transform(range.first, range.second, f);
}

template <typename RetT, typename TreeT, typename Func>
Optional<RetT> findUntilImpl(TreeT&& tree, const BoundingBox2d& area, const Func& func) {
  if (tree.empty()) {
    return {};
  }
  auto found = std::find_if(tree.qbegin(bgi::intersects(area)), tree.qend(),
                            [&func](auto& node) { return func(node.first, node.second); });
  if (found != tree.qend()) {
    return RetT(found->second);
  }
  return {};
}

template <typename RetT, typename TreeT, typename Func>
Optional<RetT> nearestUntilImpl(TreeT&& tree, const BasicPoint2d& point, const Func& func) {
  if (tree.empty()) {
    return {};
  }
  auto found = std::find_if(tree.qbegin(bgi::nearest(point, unsigned(tree.size()))), tree.qend(),
                            [&func](auto& node) { return func(node.first, node.second); });
  if (found != tree.qend()) {
    return RetT(found->second);
  }
  return {};
}

template <typename T>
void checkId(T& t) {
  if (t.id() == InvalId) {
    t.setId(utils::getId());
  } else {
    utils::registerId(t.id());
  }
}

template <typename ConstLaneletsT, typename ConstAreasT>
LaneletMapUPtr createMapFromConst(const ConstLaneletsT& fromLanelets, const ConstAreasT& fromAreas) {
  // sorry, we temporarily have to const cast our primitives back to non-const to be able to create a const map. Its not
  // beautiful but valid.
  auto lanelets = utils::transform(fromLanelets, [](const ConstLanelet& llt) {
    return Lanelet(std::const_pointer_cast<LaneletData>(llt.constData()), llt.inverted());
  });
  auto areas = utils::transform(
      fromAreas, [](const ConstArea& ar) { return Area(std::const_pointer_cast<AreaData>(ar.constData())); });
  return utils::createMap(lanelets, areas);
}
}  // namespace
}  // namespace lanelet

namespace lanelet {
template <typename T>
struct UsageLookup {
  void add(const T& prim) {
    for (const auto& elem : prim) {
      ownedLookup.insert(std::make_pair(elem, prim));
    }
  }
  std::unordered_multimap<traits::ConstPrimitiveType<traits::OwnedT<T>>, T> ownedLookup;
};

template <>
struct UsageLookup<RegulatoryElementPtr> {
  void add(const RegulatoryElementPtr& prim) {
    for (const auto& param : prim->getParameters()) {
      for (const auto& rule : param.second) {
        ownedLookup.insert(std::make_pair(rule, prim));
      }
    }
  }
  std::unordered_multimap<ConstRuleParameter, RegulatoryElementPtr> ownedLookup;
};
template <>
struct UsageLookup<Area> {
  void add(Area area) {
    auto insertElement = [area, this](auto elem) { ownedLookup.insert(std::make_pair(elem, area)); };
    utils::forEach(area.outerBound(), insertElement);

    for (const auto& innerBound : area.innerBounds()) {
      utils::forEach(innerBound, insertElement);
    }

    for (const auto& elem : area.regulatoryElements()) {
      regElemLookup.insert(std::make_pair(elem, area));
    }
  }
  std::unordered_multimap<ConstLineString3d, Area> ownedLookup;
  std::unordered_multimap<RegulatoryElementConstPtr, Area> regElemLookup;
};
template <>
struct UsageLookup<Lanelet> {
  void add(Lanelet ll) {
    ownedLookup.insert(std::make_pair(ll.leftBound(), ll));
    ownedLookup.insert(std::make_pair(ll.rightBound(), ll));
    for (const auto& elem : ll.regulatoryElements()) {
      regElemLookup.insert(std::make_pair(elem, ll));
    }
  }
  std::unordered_multimap<ConstLineString3d, Lanelet> ownedLookup;
  std::unordered_multimap<RegulatoryElementConstPtr, Lanelet> regElemLookup;
};

template <>
struct UsageLookup<Point3d> {
  void add(const Point3d& /*unused*/) {}
};

template <typename T>
struct PrimitiveLayer<T>::Tree {
  using TreeNode = std::pair<BoundingBox2d, T>;
  using RTree = bgi::rtree<TreeNode, bgi::quadratic<16>>;
  static TreeNode treeNode(const T& elem) { return {geometry::boundingBox2d(to2D(elem)), elem}; }
  explicit Tree(const PrimitiveLayer::Map& primitives) {
    std::vector<TreeNode> nodes;
    nodes.reserve(primitives.size());
    for (auto& primitive : primitives) {
      auto node = treeNode(primitive.second);
      if (!node.first.isEmpty()) {
        nodes.push_back(std::move(node));
      }
    }
    rTree = RTree(nodes);
  }

  void insert(const T& elem) {
    TreeNode node = treeNode(elem);
    if (!node.first.isEmpty()) {
      rTree.insert(node);
    }
  }
  void erase(const T& elem) {
    TreeNode node = treeNode(elem);
    if (!node.first.isEmpty()) {
      rTree.remove(node);
    }
  }
  RTree rTree;
  UsageLookup<T> usage;
};

template <>
struct PrimitiveLayer<Point3d>::Tree {
  using TreeNode = std::pair<BasicPoint2d, Point3d>;
  using RTree = bgi::rtree<TreeNode, bgi::quadratic<16>>;
  static TreeNode treeNode(const Point3d& p) { return {Point2d(p).basicPoint(), p}; }
  explicit Tree(const PrimitiveLayer::Map& primitives) {
    std::vector<TreeNode> nodes;
    nodes.reserve(primitives.size());
    std::transform(primitives.begin(), primitives.end(), std::back_inserter(nodes),
                   [](const auto& elem) { return treeNode(elem.second); });
    rTree = RTree(nodes);
  }
  void insert(const Point3d& elem) { rTree.insert(treeNode(elem)); }
  void erase(const Point3d& elem) { rTree.remove(treeNode(elem)); }
  RTree rTree;
  UsageLookup<Point3d> usage;
};

template <typename T>
PrimitiveLayer<T>::PrimitiveLayer(const PrimitiveLayer::Map& primitives)
    : elements_{primitives}, tree_{std::make_unique<Tree>(primitives)} {
  for (const auto& prim : primitives) {
    tree_->usage.add(prim.second);
  }
}

template <>
PrimitiveLayer<Lanelet>::PrimitiveLayer(const PrimitiveLayer::Map& primitives)
    : elements_{primitives}, tree_{std::make_unique<Tree>(primitives)} {
  for (const auto& prim : primitives) {
    tree_->usage.add(prim.second);
  }
}

template <>
PrimitiveLayer<Area>::PrimitiveLayer(const PrimitiveLayer::Map& primitives)
    : elements_{primitives}, tree_{std::make_unique<Tree>(primitives)} {
  for (const auto& prim : primitives) {
    tree_->usage.add(prim.second);
    utils::registerId(prim.first);
  }
}

template <typename T>
bool PrimitiveLayer<T>::exists(Id id) const {
  return id != InvalId && elements_.find(id) != elements_.end();
}

template <typename T>
typename PrimitiveLayer<T>::PrimitiveT PrimitiveLayer<T>::get(Id id) {
  if (id == InvalId) {
    throw NoSuchPrimitiveError("Tried to lookup an element with id InvalId!");
  }
  try {
    return elements_.at(id);
  } catch (std::out_of_range&) {
    throw NoSuchPrimitiveError("Failed to lookup element with id " + std::to_string(id));
  }
}

template <typename T>
typename PrimitiveLayer<T>::ConstPrimitiveT PrimitiveLayer<T>::get(Id id) const {
  if (id == InvalId) {
    throw NoSuchPrimitiveError("Tried to lookup an element with id InvalId!");
  }
  try {
    return elements_.at(id);
  } catch (std::out_of_range&) {
    throw NoSuchPrimitiveError("Failed to lookup element with id " + std::to_string(id));
  }
}

template <typename T>
void PrimitiveLayer<T>::add(const PrimitiveLayer<T>::PrimitiveT& element) {
  for (const auto& elem : element) {
    tree_->usage.ownedLookup.insert(std::make_pair(elem, element));
  }
  elements_.insert({element.id(), element});
  tree_->insert(element);
}

template <>
void PrimitiveLayer<Area>::add(const Area& area) {
  tree_->usage.add(area);
  elements_.insert({area.id(), area});
  tree_->insert(area);
}

template <>
void PrimitiveLayer<Lanelet>::add(const Lanelet& ll) {
  tree_->usage.add(ll);
  elements_.insert({ll.id(), ll});
  tree_->insert(ll);
}

template <>
void PrimitiveLayer<Point3d>::add(const Point3d& p) {
  tree_->usage.add(p);
  elements_.insert({p.id(), p});
  tree_->insert(p);
}

template <>
void PrimitiveLayer<RegulatoryElementPtr>::add(const PrimitiveLayer<RegulatoryElementPtr>::PrimitiveT& element) {
  tree_->usage.add(element);
  elements_.insert({element->id(), element});
  tree_->insert(element);
}

template <typename T>
std::vector<typename PrimitiveLayer<T>::ConstPrimitiveT> PrimitiveLayer<T>::findUsages(
    const traits::ConstPrimitiveType<traits::OwnedT<PrimitiveLayer<T>::PrimitiveT>>& primitive) const {
  return forEachMatchInMultiMap<traits::ConstPrimitiveType<typename PrimitiveLayer<T>::PrimitiveT>>(
      tree_->usage.ownedLookup, primitive, [](const auto& elem) { return traits::toConst(elem.second); });
}

template <typename T>
std::vector<typename PrimitiveLayer<T>::PrimitiveT> PrimitiveLayer<T>::findUsages(
    const traits::ConstPrimitiveType<traits::OwnedT<PrimitiveT>>& primitive) {
  return forEachMatchInMultiMap<typename PrimitiveLayer<T>::PrimitiveT>(tree_->usage.ownedLookup, primitive,
                                                                        [](const auto& elem) { return elem.second; });
}

template <>
ConstPoints3d PointLayer::findUsages(const ConstPoint3d& /*primitive*/) const {
  return ConstPoints3d();
}

template <>
Points3d PointLayer::findUsages(const ConstPoint3d& /*primitive*/) {
  return Points3d();
}

Areas AreaLayer::findUsages(const RegulatoryElementConstPtr& regElem) {
  return forEachMatchInMultiMap<Area>(tree_->usage.regElemLookup, regElem,
                                      [](const auto& elem) { return elem.second; });
}

ConstAreas AreaLayer::findUsages(const RegulatoryElementConstPtr& regElem) const {
  return forEachMatchInMultiMap<ConstArea>(tree_->usage.regElemLookup, regElem,
                                           [](const auto& elem) { return traits::toConst(elem.second); });
}

Lanelets LaneletLayer::findUsages(const RegulatoryElementConstPtr& regElem) {
  return forEachMatchInMultiMap<Lanelet>(tree_->usage.regElemLookup, regElem,
                                         [](const auto& elem) { return elem.second; });
}

ConstLanelets LaneletLayer::findUsages(const RegulatoryElementConstPtr& regElem) const {
  return forEachMatchInMultiMap<ConstLanelet>(tree_->usage.regElemLookup, regElem,
                                              [](const auto& elem) { return traits::toConst(elem.second); });
}

template <typename T>
PrimitiveLayer<T>::~PrimitiveLayer() noexcept = default;
template <typename T>
PrimitiveLayer<T>::PrimitiveLayer(PrimitiveLayer&& rhs) noexcept = default;
template <typename T>
PrimitiveLayer<T>& PrimitiveLayer<T>::operator=(PrimitiveLayer&& rhs) noexcept = default;

template <typename T>
typename PrimitiveLayer<T>::const_iterator PrimitiveLayer<T>::find(Id id) const {
  return elements_.find(id);
}

template <typename T>
typename PrimitiveLayer<T>::const_iterator PrimitiveLayer<T>::begin() const {
  return elements_.begin();
}

template <typename T>
typename PrimitiveLayer<T>::const_iterator PrimitiveLayer<T>::end() const {
  return elements_.end();
}

template <typename T>
typename PrimitiveLayer<T>::iterator PrimitiveLayer<T>::find(Id id) {
  return elements_.find(id);
}

template <typename T>
typename PrimitiveLayer<T>::iterator PrimitiveLayer<T>::begin() {
  return elements_.begin();
}

template <typename T>
typename PrimitiveLayer<T>::iterator PrimitiveLayer<T>::end() {
  return elements_.end();
}
template <typename T>
typename PrimitiveLayer<T>::ConstPrimitiveVec PrimitiveLayer<T>::search(const BoundingBox2d& area) const {
  std::vector<typename Tree::TreeNode> nodes;
  tree_->rTree.query(bgi::intersects(area), std::back_inserter(nodes));
  return getSecond<ConstPrimitiveT, typename Tree::TreeNode>(nodes);
}

template <typename T>
typename PrimitiveLayer<T>::PrimitiveVec PrimitiveLayer<T>::search(const BoundingBox2d& area) {
  std::vector<typename Tree::TreeNode> nodes;
  tree_->rTree.query(bgi::intersects(area), std::back_inserter(nodes));
  return getSecond<PrimitiveT, typename Tree::TreeNode>(nodes);
}

template <typename T>
typename PrimitiveLayer<T>::OptConstPrimitiveT PrimitiveLayer<T>::searchUntil(const BoundingBox2d& area,
                                                                              const ConstSearchFunction& func) const {
  return findUntilImpl<ConstPrimitiveT>(tree_->rTree, area, func);
}

template <typename T>
typename PrimitiveLayer<T>::OptPrimitiveT PrimitiveLayer<T>::searchUntil(const BoundingBox2d& area,
                                                                         const PrimitiveLayer::SearchFunction& func) {
  return findUntilImpl<PrimitiveT>(tree_->rTree, area, func);
}

template <typename T>
typename PrimitiveLayer<T>::ConstPrimitiveVec PrimitiveLayer<T>::nearest(const BasicPoint2d& point, unsigned n) const {
  std::vector<typename Tree::TreeNode> nodes;
  tree_->rTree.query(bgi::nearest(point, n), std::back_inserter(nodes));
  return getSecond<ConstPrimitiveT, typename Tree::TreeNode>(nodes);
}

template <typename T>
typename PrimitiveLayer<T>::PrimitiveVec PrimitiveLayer<T>::nearest(const BasicPoint2d& point, unsigned n) {
  std::vector<typename Tree::TreeNode> nodes;
  nodes.reserve(n);
  tree_->rTree.query(bgi::nearest(point, n), std::back_inserter(nodes));
  return getSecond<PrimitiveT, typename Tree::TreeNode>(nodes);
}

template <typename T>
typename PrimitiveLayer<T>::OptConstPrimitiveT PrimitiveLayer<T>::nearestUntil(const BasicPoint2d& point,
                                                                               const ConstSearchFunction& func) const {
  return nearestUntilImpl<ConstPrimitiveT>(tree_->rTree, point, func);
}

template <typename T>
typename PrimitiveLayer<T>::OptPrimitiveT PrimitiveLayer<T>::nearestUntil(const BasicPoint2d& point,
                                                                          const SearchFunction& func) {
  return nearestUntilImpl<PrimitiveT>(tree_->rTree, point, func);
}

template <typename T>
Id PrimitiveLayer<T>::uniqueId() const {
  return utils::getId();
}

LaneletMapLayers::LaneletMapLayers(const LaneletLayer::Map& lanelets, const AreaLayer::Map& areas,
                                   const RegulatoryElementLayer::Map& regulatoryElements,
                                   const PolygonLayer::Map& polygons, const LineStringLayer::Map& lineStrings,
                                   const PointLayer::Map& points)
    : laneletLayer(lanelets),
      areaLayer(areas),
      regulatoryElementLayer(regulatoryElements),
      polygonLayer(polygons),
      lineStringLayer(lineStrings),
      pointLayer(points) {}

void LaneletMap::add(Lanelet lanelet) {
  if (lanelet.id() == InvalId) {
    lanelet.setId(laneletLayer.uniqueId());
  } else if (laneletLayer.exists(lanelet.id())) {
    return;
  } else {
    utils::registerId(lanelet.id());
  }
  add(lanelet.leftBound());
  add(lanelet.rightBound());
  if (lanelet.hasCustomCenterline()) {
    add(uglyRemoveConst(lanelet.centerline()));
  }
  // assign regelems an id so that the regelem usage lookup works
  for (const auto& regelem : lanelet.regulatoryElements()) {
    if (regelem->id() == InvalId) {
      regelem->setId(regulatoryElementLayer.uniqueId());
    }
  }
  // add lanelet before adding regelem to avoid cycles.
  laneletLayer.add(lanelet);
  for (const auto& regElem : lanelet.regulatoryElements()) {
    add(regElem);
  }
}

void LaneletMap::add(Area area) {
  if (area.id() == InvalId) {
    area.setId(areaLayer.uniqueId());
  } else if (areaLayer.exists(area.id())) {
    return;
  } else {
    utils::registerId(area.id());
  }
  for (const auto& ls : area.outerBound()) {
    add(ls);
  }
  for (const auto& lss : area.innerBounds()) {
    for (const auto& ls : lss) {
      add(ls);
    }
  }
  // assign regelems an id so that the regelem usage lookup works
  for (const auto& regelem : area.regulatoryElements()) {
    if (regelem->id() == InvalId) {
      regelem->setId(regulatoryElementLayer.uniqueId());
    }
  }
  // add area before adding regelem to avoid cycles.
  areaLayer.add(area);
  for (const auto& regElem : area.regulatoryElements()) {
    add(regElem);
  }
}

void LaneletMap::add(const RegulatoryElementPtr& regElem) {
  if (!regElem) {
    throw NullptrError("Empty regulatory element passed to add()!");
  }
  if (regElem->id() == InvalId) {
    regElem->setId(regulatoryElementLayer.uniqueId());
  } else if (regulatoryElementLayer.exists(regElem->id())) {
    return;
  } else {
    utils::registerId(regElem->id());
  }
  AssignIdVisitor setIdVisitor(this);
  for (const auto& elems : *regElem) {
    for (const auto& elem : elems.second) {
      boost::apply_visitor(setIdVisitor, elem);
    }
  }
  regulatoryElementLayer.add(regElem);

  AddVisitor visitor(this);
  regElem->applyVisitor(visitor);
}

void LaneletMap::add(Polygon3d polygon) {
  if (polygon.id() == InvalId) {
    polygon.setId(polygonLayer.uniqueId());
  } else if (polygonLayer.exists(polygon.id())) {
    return;
  } else {
    utils::registerId(polygon.id());
  }
  for (const auto& p : polygon) {
    add(p);
  }
  polygonLayer.add(polygon);
}

void LaneletMap::add(LineString3d lineString) {
  if (lineString.id() == InvalId) {
    lineString.setId(lineStringLayer.uniqueId());
  } else if (lineStringLayer.exists(lineString.id())) {
    return;
  } else {
    utils::registerId(lineString.id());
  }
  for (const auto& p : lineString) {
    add(p);
  }
  lineStringLayer.add(lineString);
}

void LaneletMap::add(Point3d point) {
  if (point.id() == InvalId) {
    point.setId(pointLayer.uniqueId());
  } else if (pointLayer.exists(point.id())) {
    return;
  } else {
    utils::registerId(point.id());
  }
  pointLayer.add(point);
}

void LaneletSubmap::add(Lanelet lanelet) {
  checkId(lanelet);
  utils::forEach(lanelet.regulatoryElements(), [&](auto& regElem) { this->trackParameters(*regElem); });
  laneletLayer.add(lanelet);
}

void LaneletSubmap::add(Area area) {
  checkId(area);
  utils::forEach(area.regulatoryElements(), [&](auto& regElem) { this->trackParameters(*regElem); });
  areaLayer.add(area);
}

void LaneletSubmap::add(const RegulatoryElementPtr& regElem) {
  checkId(*regElem);
  trackParameters(*regElem);
  regulatoryElementLayer.add(regElem);
}

void LaneletSubmap::add(Polygon3d polygon) {
  checkId(polygon);
  polygonLayer.add(polygon);
}

void LaneletSubmap::add(LineString3d lineString) {
  checkId(lineString);
  lineStringLayer.add(lineString);
}

void LaneletSubmap::add(Point3d point) {
  checkId(point);
  pointLayer.add(point);
}

LaneletMapConstPtr LaneletSubmap::laneletMap() const {
  auto map = createMapFromConst(laneletLayer, areaLayer);
  auto add = [&map](auto& prim) {
    using T = std::decay_t<decltype(prim)>;
    map->add(typename traits::PrimitiveTraits<T>::MutableType(
        std::const_pointer_cast<typename traits::PrimitiveTraits<T>::DataType>(prim.constData())));
  };
  auto addLs = [&map](auto& prim) {
    map->add(LineString3d(std::const_pointer_cast<LineStringData>(prim.constData()), prim.inverted()));
  };
  auto addRe = [&map](auto& prim) { map->add(std::const_pointer_cast<RegulatoryElement>(prim)); };
  utils::forEach(regulatoryElementLayer, addRe);
  utils::forEach(polygonLayer, add);
  utils::forEach(lineStringLayer, addLs);
  utils::forEach(pointLayer, add);
  return map;
}

LaneletMapUPtr LaneletSubmap::laneletMap() {
  auto map =
      utils::createMap(Lanelets{laneletLayer.begin(), laneletLayer.end()}, Areas{areaLayer.begin(), areaLayer.end()});
  auto add = [&](auto& val) { map->add(val); };
  utils::forEach(regulatoryElementLayer, add);
  utils::forEach(polygonLayer, add);
  utils::forEach(lineStringLayer, add);
  utils::forEach(pointLayer, add);
  return map;
}

void LaneletSubmap::trackParameters(const RegulatoryElement& regelem) {
  using LLorAreas = std::vector<boost::variant<ConstLanelet, ConstArea>>;
  class AddToLLorAreaVisitor : public RuleParameterVisitor {
   public:
    explicit AddToLLorAreaVisitor(LLorAreas& llOrAreas) : llOrAreas_{&llOrAreas} {}
    void operator()(const ConstWeakLanelet& wll) override {
      if (!wll.expired()) {
        llOrAreas_->emplace_back(wll.lock());
      }
    }
    void operator()(const ConstWeakArea& wa) override {
      if (!wa.expired()) {
        llOrAreas_->emplace_back(wa.lock());
      }
    }

   private:
    LLorAreas* llOrAreas_{};
  };
  AddToLLorAreaVisitor v{regelemObjects_};
  regelem.applyVisitor(v);
}

template class PrimitiveLayer<Area>;
template class PrimitiveLayer<Lanelet>;
template class PrimitiveLayer<Point3d>;
template class PrimitiveLayer<LineString3d>;
template class PrimitiveLayer<Polygon3d>;
template class PrimitiveLayer<RegulatoryElementPtr>;

namespace utils {
template <typename PrimitiveT>
std::vector<ConstLayerPrimitive<PrimitiveT>> findUsages(const PrimitiveLayer<PrimitiveT>& layer, Id id) {
  std::vector<ConstLayerPrimitive<PrimitiveT>> usages;
  std::copy_if(layer.begin(), layer.end(), std::back_inserter(usages),
               [&id](const auto& elem) { return utils::has(elem, id); });
  return usages;
}

template std::vector<ConstLayerPrimitive<Lanelet>> findUsages<Lanelet>(const PrimitiveLayer<Lanelet>&, Id);
template std::vector<ConstLayerPrimitive<LineString3d>> findUsages<LineString3d>(const PrimitiveLayer<LineString3d>&,
                                                                                 Id);
template std::vector<ConstLayerPrimitive<Polygon3d>> findUsages<Polygon3d>(const PrimitiveLayer<Polygon3d>&, Id);
template std::vector<ConstLayerPrimitive<RegulatoryElementPtr>> findUsages<RegulatoryElementPtr>(
    const PrimitiveLayer<RegulatoryElementPtr>&, Id);

ConstLanelets findUsagesInLanelets(const LaneletMapLayers& map, const ConstPoint3d& p) {
  auto ls = map.lineStringLayer.findUsages(p);
  auto llts = utils::concatenate(ls, [&map](const auto& ls) { return map.laneletLayer.findUsages(ls); });
  auto lltsInv = utils::concatenate(ls, [&map](const auto& ls) { return map.laneletLayer.findUsages(ls.invert()); });
  llts.insert(llts.end(), lltsInv.begin(), lltsInv.end());
  auto remove = std::unique(llts.begin(), llts.end());
  llts.erase(remove, llts.end());
  return llts;
}

ConstAreas findUsagesInAreas(const LaneletMapLayers& map, const ConstPoint3d& p) {
  auto ls = map.lineStringLayer.findUsages(p);
  auto areas = utils::concatenate(ls, [&map](const auto& ls) { return map.areaLayer.findUsages(ls); });
  auto areasInv = utils::concatenate(ls, [&map](const auto& ls) { return map.areaLayer.findUsages(ls.invert()); });
  areas.insert(areas.end(), areasInv.begin(), areasInv.end());
  auto remove = std::unique(areas.begin(), areas.end());
  areas.erase(remove, areas.end());
  return areas;
}

LaneletMapUPtr createMap(const Points3d& fromPoints) {
  return std::make_unique<LaneletMap>(LaneletLayer::Map(), AreaLayer::Map(), RegulatoryElementLayer::Map(),
                                      PolygonLayer::Map(), LineStringLayer::Map(), toMap(fromPoints));
}

LaneletMapUPtr createMap(const LineStrings3d& fromLineStrings) {
  auto points = utils::concatenateRange(fromLineStrings, [](auto ls) { return std::make_pair(ls.begin(), ls.end()); });
  return std::make_unique<LaneletMap>(LaneletLayer::Map(), AreaLayer::Map(), RegulatoryElementLayer::Map(),
                                      PolygonLayer::Map(), toMap(fromLineStrings), toMap(points));
}

LaneletMapUPtr createMap(const Lanelets& fromLanelets, const Areas& fromAreas) {
  LineStrings3d ls;
  ls.reserve(fromLanelets.size() * 2 + fromAreas.size() * 3);
  for (auto ll : fromLanelets) {
    ls.push_back(ll.leftBound());
    ls.push_back(ll.rightBound());
  }
  auto appendMultiLs = [&ls](auto& lsappend) {
    std::for_each(lsappend.begin(), lsappend.end(), [&ls](auto& l) { ls.push_back(l); });
  };
  for (auto area : fromAreas) {
    appendMultiLs(area.outerBound());
    for (auto inner : area.innerBounds()) {
      appendMultiLs(inner);
    }
  }

  auto points = utils::concatenateRange(ls, [](auto ls) { return std::make_pair(ls.begin(), ls.end()); });
  auto regelems = utils::concatenateRange(fromLanelets, [](Lanelet llt) {
    const auto& regelems = llt.regulatoryElements();
    return std::make_pair(regelems.begin(), regelems.end());
  });
  auto areaRegelems = utils::concatenateRange(fromAreas, [](Area area) {
    const auto& regelems = area.regulatoryElements();
    return std::make_pair(regelems.begin(), regelems.end());
  });
  auto map = std::make_unique<LaneletMap>(toMap(fromLanelets), toMap(fromAreas), RegulatoryElementLayer::Map(),
                                          PolygonLayer::Map(), toMap(ls), toMap(points));
  for (const auto& regelem : regelems) {
    map->add(regelem);
  }
  for (const auto& regelem : areaRegelems) {
    map->add(regelem);
  }
  return map;
}

LaneletMapUPtr createMap(const Lanelets& fromLanelets) { return createMap(fromLanelets, {}); }

LaneletMapUPtr createMap(const Areas& fromAreas) { return createMap({}, fromAreas); }

LaneletMapUPtr createMap(const Polygons3d& fromPolygons) {
  auto points = utils::concatenateRange(fromPolygons, [](auto ls) { return std::make_pair(ls.begin(), ls.end()); });
  return std::make_unique<LaneletMap>(LaneletLayer::Map(), AreaLayer::Map(), RegulatoryElementLayer::Map(),
                                      toMap(fromPolygons), LineStringLayer::Map(), toMap(points));
}

LaneletMapConstUPtr createConstMap(const ConstLanelets& fromLanelets, const ConstAreas& fromAreas) {
  return createMapFromConst(fromLanelets, fromAreas);
}

namespace {
std::atomic<Id> currId{1000};
}  // namespace

Id getId() { return currId++; }

void registerId(Id id) {
  // this fuzz is needed because we want to be thread safe
  Id newId = id + 1;
  Id current = currId.load();
  while (current < newId && !currId.compare_exchange_weak(current, newId)) {
  }
}

LaneletSubmapUPtr createSubmap(const Points3d& fromPoints) {
  return std::make_unique<LaneletSubmap>(LaneletLayer::Map(), AreaLayer::Map(), RegulatoryElementLayer::Map(),
                                         PolygonLayer::Map(), LineStringLayer::Map(), toMap(fromPoints));
}

LaneletSubmapUPtr createSubmap(const LineStrings3d& fromLineStrings) {
  return std::make_unique<LaneletSubmap>(LaneletLayer::Map(), AreaLayer::Map(), RegulatoryElementLayer::Map(),
                                         PolygonLayer::Map(), toMap(fromLineStrings), PointLayer::Map());
}

LaneletSubmapUPtr createSubmap(const Polygons3d& fromPolygons) {
  return std::make_unique<LaneletSubmap>(LaneletLayer::Map(), AreaLayer::Map(), RegulatoryElementLayer::Map(),
                                         toMap(fromPolygons), LineStringLayer::Map(), PointLayer::Map());
}

LaneletSubmapUPtr createSubmap(const Lanelets& fromLanelets) { return createSubmap(fromLanelets, {}); }

LaneletSubmapUPtr createSubmap(const Areas& fromAreas) { return createSubmap({}, fromAreas); }

LaneletSubmapUPtr createSubmap(const Lanelets& fromLanelets, const Areas& fromAreas) {
  auto map = std::make_unique<LaneletSubmap>(toMap(fromLanelets), toMap(fromAreas), RegulatoryElementLayer::Map(),
                                             PolygonLayer::Map(), LineStringLayer::Map(), PointLayer::Map());
  for (const auto& ll : fromLanelets) {
    utils::forEach(ll.regulatoryElements(), [&](auto& regelem) { map->trackParameters(*regelem); });
  }
  for (const auto& ar : fromAreas) {
    utils::forEach(ar.regulatoryElements(), [&](auto& regelem) { map->trackParameters(*regelem); });
  }
  return map;
}

LaneletSubmapConstUPtr createConstSubmap(const ConstLanelets& fromLanelets, const ConstAreas& fromAreas) {
  // sorry, we temporarily have to const cast our primitives back to non-const to be able to create a const map. Its not
  // beautiful but valid.
  auto lanelets = utils::transform(fromLanelets, [](const ConstLanelet& llt) {
    return Lanelet(std::const_pointer_cast<LaneletData>(llt.constData()), llt.inverted());
  });
  auto areas = utils::transform(
      fromAreas, [](const ConstArea& ar) { return Area(std::const_pointer_cast<AreaData>(ar.constData())); });
  return utils::createSubmap(lanelets, areas);
}

}  // namespace utils

namespace geometry {
namespace {
template <typename T>
class NSmallestElements {
 public:
  explicit NSmallestElements(size_t n) : n_{n} { values_.reserve(n); }
  bool insert(double measure, T value) {
    auto pos =
        std::lower_bound(values_.begin(), values_.end(), measure, [](auto& v1, double v2) { return v1.first < v2; });
    if (pos != values_.end() || values_.size() < n_) {
      if (values_.size() >= n_) {
        values_.pop_back();
      }
      values_.emplace(pos, measure, value);
      return true;
    }
    return false;
  }
  const auto& values() const { return values_; }
  auto moveValues() { return std::move(values_); }
  bool full() const { return values_.size() >= n_; }
  bool empty() const { return values_.empty(); }

 private:
  std::vector<std::pair<double, T>> values_;
  size_t n_;
};

template <typename RetT, typename LayerT>
auto findNearestImpl(LayerT&& map, const BasicPoint2d& pt, unsigned count) {
  // Idea: nearestUntil passes us elements with increasing bounding box
  // distance.
  // If this distance is greater than the actual distance (between primitives)
  // of the count-th furthest current element, then we can stop, because all
  // succeding elements will be even further away.
  NSmallestElements<RetT> closest(count);
  auto searchFunction = [&closest, &pt](auto& box, const RetT& prim) {
    auto dBox = distance(box, pt);
    if (closest.full() && dBox > closest.values().back().first) {
      return true;
    }
    closest.insert(distance2d(prim, pt), prim);
    return false;
  };
  map.nearestUntil(pt, searchFunction);
  return closest.moveValues();
}
}  // namespace

template <typename PrimT>
std::vector<std::pair<double, PrimT>> findNearest(PrimitiveLayer<PrimT>& map, const BasicPoint2d& pt, unsigned count) {
  return findNearestImpl<PrimT>(map, pt, count);
}

template <typename PrimT>
std::vector<std::pair<double, traits::ConstPrimitiveType<PrimT>>> findNearest(const PrimitiveLayer<PrimT>& map,
                                                                              const BasicPoint2d& pt, unsigned count) {
  return findNearestImpl<traits::ConstPrimitiveType<PrimT>>(map, pt, count);
}

// instanciate
// clang-format off
// NOLINTNEXTLINE
#define INSTANCIATE_FIND_NEAREST(PRIM) template std::vector<std::pair<double, PRIM>> findNearest<PRIM>(PrimitiveLayer<PRIM>&, const BasicPoint2d&, unsigned)
// NOLINTNEXTLINE
#define INSTANCIATE_CONST_FIND_NEAREST(PRIM, CPRIM) template std::vector<std::pair<double, CPRIM>> findNearest<PRIM>(const PrimitiveLayer<PRIM>&, const BasicPoint2d&, unsigned)
// clang-format on
INSTANCIATE_FIND_NEAREST(Lanelet);
INSTANCIATE_FIND_NEAREST(Area);
INSTANCIATE_FIND_NEAREST(LineString3d);
INSTANCIATE_FIND_NEAREST(RegulatoryElementPtr);
INSTANCIATE_FIND_NEAREST(Polygon3d);
INSTANCIATE_FIND_NEAREST(Point3d);
#undef INSTANCIATE_FIND_NEAREST

INSTANCIATE_CONST_FIND_NEAREST(Lanelet, ConstLanelet);
INSTANCIATE_CONST_FIND_NEAREST(Area, ConstArea);
INSTANCIATE_CONST_FIND_NEAREST(LineString3d, ConstLineString3d);
INSTANCIATE_CONST_FIND_NEAREST(RegulatoryElementPtr, RegulatoryElementConstPtr);
INSTANCIATE_CONST_FIND_NEAREST(Polygon3d, ConstPolygon3d);
INSTANCIATE_CONST_FIND_NEAREST(Point3d, ConstPoint3d);
#undef INSTANCIATE_CONST_FIND_NEAREST
}  // namespace geometry
}  // namespace lanelet
