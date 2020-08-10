#pragma once

#include <unordered_map>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/Area.h"
#include "lanelet2_core/primitives/BoundingBox.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/primitives/RegulatoryElement.h"
#include "lanelet2_core/utility/Utilities.h"

namespace lanelet {
namespace internal {
template <typename T>
struct SearchBox {
  using Type = BoundingBox2d;
};
template <>
struct SearchBox<Point3d> {
  using Type = BasicPoint2d;
};
template <typename T>
using SearchBoxT = typename SearchBox<T>::Type;
}  // namespace internal
   /**
    * @brief Each primitive in lanelet2 has its own layer in the map.
    *
    * This template class defines the common interface for these layers.
    * It is only implemented for lanelet primitives and should not be
    * instanciated with any other type.
    *
    * Layers are an integral part of LaneletMap, they can not be created
    * or modified without a LaneletMap object.
    *
    * Internally, the elements are identified by their id, therefore it is
    * absolutely important that an id is unique within one layer.
    */
template <typename T>
class PrimitiveLayer {
 public:
  using PrimitiveT = T;
  using ConstPrimitiveT = traits::ConstPrimitiveType<T>;
  using Map = std::unordered_map<Id, T>;
  using ConstPrimitiveVec = std::vector<ConstPrimitiveT>;
  using PrimitiveVec = std::vector<PrimitiveT>;
  using OptConstPrimitiveT = Optional<ConstPrimitiveT>;
  using OptPrimitiveT = Optional<PrimitiveT>;

  //! iterator that gives the primitive when dereferencing, not a std:pair
  using iterator =  //  NOLINT
      internal::TransformIterator<typename Map::iterator, PrimitiveT, internal::PairConverter<PrimitiveT>>;

  //! iterator that gives a const primitive, not a std::pair
  using const_iterator =  // NOLINT
      internal::TransformIterator<typename Map::const_iterator, const ConstPrimitiveT,
                                  internal::PairConverter<const ConstPrimitiveT>>;
  PrimitiveLayer(const PrimitiveLayer& rhs) = delete;
  PrimitiveLayer& operator=(const PrimitiveLayer& rhs) = delete;

  /**
   * @brief checks whether an element exists in this layer
   * @param id the id identifying the element
   * @return true if element exists
   */
  bool exists(Id id) const;

  /**
   * @brief returns an element for this id if it exists
   * @param id the id identifying the element
   * @throws NoSuchPrimitiveError if the element does not exist
   * @return the const version of the element
   */
  ConstPrimitiveT get(Id id) const;

  /**
   * @brief returns an element for this id if it exists
   * @param id the id identifying the element
   * @throws NoSuchPrimitiveError if the element does not exist
   * @return the non-const version of the element
   */
  PrimitiveT get(Id id);

  /**
   * @brief find returns an iterator to an element if it exists
   * @param id id to look for
   * @return iterator to the element or end()
   *
   * Note that dereferencing the iterator will give you only the primitive, not
   * a std::pair as usual with normal std::maps!
   */
  const_iterator find(Id id) const;

  /**
   * @brief finds usages of an owned type within this layer
   * @see utils::findUsages
   *
   * Finds e.g. points owned by linestrings in the lanelet layer.
   *
   * The relations are stored by a map internally, so this is just a fast map
   * lookup.
   */
  std::vector<ConstPrimitiveT> findUsages(
      const traits::ConstPrimitiveType<traits::OwnedT<PrimitiveT>>& primitive) const;

  /**
   * @brief finds usages of an owned type within this layer
   *
   * This is the non-const version to find usages of a primitive in a layer.
   */
  std::vector<PrimitiveT> findUsages(const traits::ConstPrimitiveType<traits::OwnedT<PrimitiveT>>& primitive);

  /**
   * @brief iterator to beginning of the elements (not ordered by id!)
   * @return the iterator
   */
  const_iterator begin() const;

  /**
   * @brief iterator to end of the elements
   * @return the iterator
   */
  const_iterator end() const;

  /**
   * @brief non-const version of finding an element
   * @param id the id to look for
   * @return iterator to the matching element or end()
   */
  iterator find(Id id);

  /**
   * @brief iterator to beginning
   * @return the iterator
   */
  iterator begin();

  /**
   * @brief iterator to end
   * @return the iterator
   */
  iterator end();

  /**
   * @brief returns whether this layer contains something
   * @return true if no elements
   */
  bool empty() const { return elements_.empty(); }

  /**
   * @brief returns number of elements in this layer
   * @return number of elements
   */
  size_t size() const { return elements_.size(); }

  using ConstSearchFunction = std::function<bool(const internal::SearchBoxT<T>& box, const ConstPrimitiveT& prim)>;
  using SearchFunction = std::function<bool(const internal::SearchBoxT<T>& box, const PrimitiveT& prim)>;

  /**
   * @brief searches for elements within a search area
   * @param area the search area in the map
   * @return a immutable list of matching elements
   *
   * The search is done by comparing bounding boxes, therefore this function
   * might return false positives because the elements do not intersect with the
   * area, but their bounding boxes do. To sort these out use
   * lanelet::geometry::intersects or lanelet::geometry::overlaps.
   *
   * Note that this function is also implemented for the point layer for
   * consistency, even if it does not make sense for this type. It will always
   * return an empty vector.
   */
  ConstPrimitiveVec search(const BoundingBox2d& area) const;
  PrimitiveVec search(const BoundingBox2d& area);

  /**
   * @brief searches within search area until a search function returns true.
   * @return the element for which the function returned true
   * @see nearestUntil
   *
   * Starting by the object with the closest bounding box, searchUntil will pass
   * primitives to func until the result is true. This is the returned object.
   * If no such object exists, the Optional will be empty.
   */
  OptConstPrimitiveT searchUntil(const BoundingBox2d& area, const ConstSearchFunction& func) const;
  OptPrimitiveT searchUntil(const BoundingBox2d& area, const SearchFunction& func);

  /**
   * @brief search for the n nearest elements to a point
   * @param point the point for the query
   * @param n number of elements returned
   * @return the element whose *bounding boxes* are closest to the point
   * @see nearestUntil, lanelet::geometry::findNearest
   *
   * Note that this function does not yield accurate results for all primitives
   * except points. For all other primitives, "nearest" is determined by the
   * bounding box of the object. For regulatory elements, this will be the
   * bounding box of all parameters of the regulatory element. For Lanelets, it
   * will be the bounding box alround the polygon.
   *
   * If you are not happy with this, have a look at nearestUntil, or one of the
   * free functions, like geometry::findNearest.
   */
  ConstPrimitiveVec nearest(const BasicPoint2d& point, unsigned n) const;
  PrimitiveVec nearest(const BasicPoint2d& point, unsigned n);
  ConstPrimitiveVec nearest(const Point2d& point, unsigned n) const { return nearest(point.basicPoint(), n); }
  PrimitiveVec nearest(const Point2d& point, unsigned n) { return nearest(point.basicPoint(), n); }

  /**
   * @brief repeatedly calls a user-defined predicate until it returns true.
   * @return the element for which the function returned true. If it was never
   * true, an empty Optional is returned.
   *
   * Starting at the primitive with the closest bounding-box distance to the
   * query point iteratively calls func until true is returned. This can be used
   * for iterative queries.
   */
  OptConstPrimitiveT nearestUntil(const BasicPoint2d& point, const ConstSearchFunction& func) const;
  OptPrimitiveT nearestUntil(const BasicPoint2d& point, const SearchFunction& func);
  OptConstPrimitiveT nearestUntil(const ConstPoint2d& point, const ConstSearchFunction& func) const {
    return nearestUntil(point.basicPoint2d(), func);
  }
  OptPrimitiveT nearestUntil(const Point2d& point, const SearchFunction& func) {
    return nearestUntil(point.basicPoint2d(), func);
  }

  /**
   * @brief returns a unique id. it is guaranteed that the id is not used within
   * this layer
   * @return an unused id
   *
   * Several calls to uniqueId might or might not produce the same Id, as long
   * as no element with this Id was added to the layer
   */
  Id uniqueId() const;

 protected:
  friend class LaneletMap;  // only the map can create or modify layers
  friend class LaneletMapLayers;
  friend class LaneletSubmap;
  explicit PrimitiveLayer(const Map& primitives = Map());
  PrimitiveLayer(PrimitiveLayer&& rhs) noexcept;
  PrimitiveLayer& operator=(PrimitiveLayer&& rhs) noexcept;
  ~PrimitiveLayer() noexcept;

  void add(const PrimitiveT& element);
  void remove(Id element);

  // NOLINTNEXTLINE
  Map elements_;  //!< the list of elements in this layer
  struct Tree;
  // NOLINTNEXTLINE
  std::unique_ptr<Tree> tree_;  //!< Hides boost trees from you/the compiler
};

// we need this ifndef for LaneletMap.cpp
#ifndef LANELET_LAYER_DEFINITION
extern template class PrimitiveLayer<Area>;
extern template class PrimitiveLayer<Polygon3d>;
extern template class PrimitiveLayer<Lanelet>;
extern template class PrimitiveLayer<LineString3d>;
extern template class PrimitiveLayer<Point3d>;
extern template class PrimitiveLayer<RegulatoryElementPtr>;
#endif

//! Specialized map layer for Area.
class AreaLayer : public PrimitiveLayer<Area> {
 public:
  using PrimitiveLayer::findUsages;
  AreaLayer() = default;
  ~AreaLayer() = default;
  AreaLayer(const AreaLayer&) = delete;
  AreaLayer operator=(AreaLayer&) = delete;
  Areas findUsages(const RegulatoryElementConstPtr& regElem);
  ConstAreas findUsages(const RegulatoryElementConstPtr& regElem) const;

 private:
  friend class LaneletMap;
  friend class LaneletMapLayers;
  friend class LaneletSubmap;
  using PrimitiveLayer<Area>::PrimitiveLayer;
  AreaLayer(AreaLayer&& rhs) noexcept = default;
  AreaLayer& operator=(AreaLayer&& rhs) noexcept = default;
};

//! Specialized map layer for Lanelet.
class LaneletLayer : public PrimitiveLayer<Lanelet> {
 public:
  using PrimitiveLayer::findUsages;
  LaneletLayer() = default;
  ~LaneletLayer() = default;
  LaneletLayer(const LaneletLayer&) = delete;
  LaneletLayer operator=(LaneletLayer&) = delete;
  Lanelets findUsages(const RegulatoryElementConstPtr& regElem);
  ConstLanelets findUsages(const RegulatoryElementConstPtr& regElem) const;

 private:
  friend class LaneletMap;
  friend class LaneletMapLayers;
  friend class LaneletSubmap;
  using PrimitiveLayer<Lanelet>::PrimitiveLayer;
  LaneletLayer(LaneletLayer&& rhs) noexcept = default;
  LaneletLayer& operator=(LaneletLayer&& rhs) noexcept = default;
};

using PolygonLayer = PrimitiveLayer<Polygon3d>;
using LineStringLayer = PrimitiveLayer<LineString3d>;
using PointLayer = PrimitiveLayer<Point3d>;
using RegulatoryElementLayer = PrimitiveLayer<RegulatoryElementPtr>;

/**
 * @brief Container for all layers of a lanelet map. Used by both LaneletMap and LaneletSubmap
 */
class LaneletMapLayers {
 public:
  LaneletMapLayers() = default;
  LaneletMapLayers(LaneletMapLayers&& rhs) noexcept = default;
  LaneletMapLayers& operator=(LaneletMapLayers&& rhs) noexcept = default;
  LaneletMapLayers(const LaneletMapLayers& rhs) = delete;
  LaneletMapLayers& operator=(const LaneletMapLayers& rhs) = delete;
  ~LaneletMapLayers() noexcept = default;

  /**
   * @brief Construct from already initialized layers
   * @param lanelets new lanelet layer
   * @param areas new area layer
   * @param regulatoryElements new regulatoryElement layer
   * @param polygons polygon layer
   * @param lineStrings linesting layer
   * @param points point layer
   *
   * Constructs a map from its individual elements for the layers.
   * Unlike with the add functions you are responsible that e.g. all points of
   * a linestring are in the point layer. However, this is the most efficient
   * way to create a map because this will result in the most efficient RTree
   * structure for fastest lookups.
   */
  LaneletMapLayers(const LaneletLayer::Map& lanelets, const AreaLayer::Map& areas,
                   const RegulatoryElementLayer::Map& regulatoryElements, const PolygonLayer::Map& polygons,
                   const LineStringLayer::Map& lineStrings, const PointLayer::Map& points);

  //! Returns whether all layers of this object are empty
  bool empty() const noexcept {
    return laneletLayer.empty() && areaLayer.empty() && regulatoryElementLayer.empty() && polygonLayer.empty() &&
           lineStringLayer.empty() && pointLayer.empty();
  }

  //! Returns the total number of elements in all layers
  size_t size() const noexcept {
    return laneletLayer.size() + areaLayer.size() + regulatoryElementLayer.size() + polygonLayer.size() +
           lineStringLayer.size() + pointLayer.size();
  }

  LaneletLayer laneletLayer;                      //!< access to the lanelets within this map
  AreaLayer areaLayer;                            //!< access to areas
  RegulatoryElementLayer regulatoryElementLayer;  //!< access to regElems
  PolygonLayer polygonLayer;                      //!< access to the polygons
  LineStringLayer lineStringLayer;                //!< access to the lineStrings
  PointLayer pointLayer;                          //!< access to the points
};

/**
 * @brief Basic element for accessing and managing the elements of a map.
 *
 * The map is divided in individual layers, one for each primitive in lanelet2.
 * Each layer offers efficient functions to find elements by its id or spacial
 * position. A LaneletMap can not be copied because maps are unique. You can
 * only std::move them.
 *
 * A LaneletMap is designed to be *always* self contained. This means it always contains all elements that are used by
 * any element in the map. If you add a Lanelet, all its LineString boundaries, all RegulatoryElements, all things
 * referenced by the regulatory elements are added to the map as well. This also means that if a lanelet has regulatory
 * elements that in turn also reference lanelets, these lanelets will also be added to the map. If this behaviour is not
 * what you want, consider using a LaneletSubmap.
 */
class LaneletMap : public LaneletMapLayers {
 public:
  using LaneletMapLayers::LaneletMapLayers;

  /**
   * @brief adds a lanelet and all the elements it owns to the map
   * @throws InvalidInputError if lanelet has a reglatory element without
   * members
   *
   * If the lanelet or elements owned by the lanelet have InvalId as Id, they
   * will be assigned a new, unique id. Otherwise you are responsible for making
   * sure that the id has not already been for a different element.
   */
  void add(Lanelet lanelet);

  /**
   * @brief adds an area and all the elements it owns
   * @param area the area to add
   * @throws InvalidInputError if area has a reglatory element without members
   *
   * If the area or elements owned by it have InvalId as Id, they
   * will be assigned a new, unique id. Otherwise you are responsible for making
   * sure that the id has not already been for a different element.
   */
  void add(Area area);

  /**
   * @brief adds a new regulatory element and all elements it owns to the map.
   * @throws NullptrError if regElem is a nullptr
   * @throws InvalidInputError if regElem has no members
   *
   * If the regulatory elemnet or elements owned it lanelet have InvalId as
   * Id, they will be assigned a new, unique id. Otherwise you are responsible
   * for making sure that the id has not already been for a different element.
   */
  void add(const RegulatoryElementPtr& regElem);

  /**
   * @brief adds a new polygon and all elements it owns to the map.
   *
   * If the polygon or elements owned by it have InvalId as Id, they
   * will be assigned a new, unique id. Otherwise you are responsible for making
   * sure that the id has not already been for a different element.
   */
  void add(Polygon3d polygon);

  /**
   * @brief adds a new lineString and all elements it owns to the map.
   *
   * If the lineString or elements owned by it have InvalId as Id, they
   * will be assigned a new, unique id. Otherwise you are responsible for making
   * sure that the id has not already been for a different element.
   */
  void add(LineString3d lineString);

  /**
   * @brief adds a new point and all elements it owns to the map.
   *
   * If the point or elements owned by it have InvalId as Id, they
   * will be assigned a new, unique id. Otherwise you are responsible for making
   * sure that the id has not already been for a different element.
   */
  void add(Point3d point);
};

/**
 * @brief A LaneletSubmap only contains the elemets that have be expleicitly added to it.
 *
 * This class is similar to a LaneletMap but with one fundamental difference: It is not self-contained. When add is
 * called with an object *only this object* is added to the map and nothing else like e.g. its subobjects. But you can
 * always convert this object back to a laneletMap by calling "LaneletSubmap::laneletMap", which is of course a costly
 * operation.
 *
 * If you want to have a function that can operate on both LaneletMap and LaneletSubmap, consider using their common
 * base class LaneletMapLayers as an input parameter for it.
 */
class LaneletSubmap : public LaneletMapLayers {
 public:
  LaneletSubmap() = default;
  using LaneletMapLayers::LaneletMapLayers;

  //! Constructs a submap from a moved-from LaneletMap
  explicit LaneletSubmap(LaneletMap&& rhs) noexcept : LaneletMapLayers{std::move(rhs)} {}
  LaneletSubmap& operator=(LaneletMap&& rhs) {
    static_cast<LaneletMapLayers&>(*this) = std::move(rhs);
    return *this;
  }

  /**
   * @brief adds a lanelet to the submap
   * @throws InvalidInputError if lanelet has a reglatory element without
   * members
   *
   * If the lanelet or elements owned by the lanelet have InvalId as Id, they
   * will be assigned a new, unique id. Otherwise you are responsible for making
   * sure that the id has not already been for a different element.
   */
  void add(Lanelet lanelet);

  //! adds an area
  void add(Area area);

  //! adds a new regulatory element to the submap.
  void add(const RegulatoryElementPtr& regElem);

  //! adds a new polygon to the submap
  void add(Polygon3d polygon);

  //! adds a new lineString the submap
  void add(LineString3d lineString);

  //! adds a new point to the submap
  void add(Point3d point);

  //! Converts this into a fully valid lanelet map
  LaneletMapConstPtr laneletMap() const;
  LaneletMapUPtr laneletMap();

  //! In order to let areas and lanelets get out of scope, this function ensures they stay alive. LaneletSubmap::add
  //! already handles this for you, so there is usually no need to call this.
  void trackParameters(const RegulatoryElement& regelem);

 private:
  //! In order to not let lanelets/areas referenced by regelems get out of scope, we keep a reference to them here
  std::vector<boost::variant<ConstLanelet, ConstArea>> regelemObjects_;
};

namespace utils {
template <typename PrimitiveT>
using ConstLayerPrimitive = typename PrimitiveLayer<PrimitiveT>::ConstPrimitiveT;

//! Efficiently create a LaneletMap from a vector of things. All elements must have a valid id!
LaneletMapUPtr createMap(const Points3d& fromPoints);
LaneletMapUPtr createMap(const LineStrings3d& fromLineStrings);
LaneletMapUPtr createMap(const Polygons3d& fromPolygons);
LaneletMapUPtr createMap(const Lanelets& fromLanelets);
LaneletMapUPtr createMap(const Areas& fromAreas);
LaneletMapUPtr createMap(const Lanelets& fromLanelets, const Areas& fromAreas);
LaneletMapConstUPtr createConstMap(const ConstLanelets& fromLanelets, const ConstAreas& fromAreas);

//! Efficiently create a LaneletSubmap from a vector of things. All elements must have a valid id!
LaneletSubmapUPtr createSubmap(const Points3d& fromPoints);
LaneletSubmapUPtr createSubmap(const LineStrings3d& fromLineStrings);
LaneletSubmapUPtr createSubmap(const Polygons3d& fromPolygons);
LaneletSubmapUPtr createSubmap(const Lanelets& fromLanelets);
LaneletSubmapUPtr createSubmap(const Areas& fromAreas);
LaneletSubmapUPtr createSubmap(const Lanelets& fromLanelets, const Areas& fromAreas);
LaneletSubmapConstUPtr createConstSubmap(const ConstLanelets& fromLanelets, const ConstAreas& fromAreas);

/**
 * @brief returns a unique id by incrementing a counter each time this is
 * called.
 *
 * Useful for creating a new map with primitives. As long as all the ids are
 * generated using this function, it is guaranteed that there will be no
 * duplicates. This is different from PrimitiveLayer::uniqueId, which only
 * guarantees that an Id is unique within this layer.
 *
 * This is threa safe.
 */
Id getId();

//! Register an id as being in use. This id will no longer be returned by getId.
//! Function is thread safe.
void registerId(Id id);

template <typename PrimitiveT>
std::vector<ConstLayerPrimitive<PrimitiveT>> findUsages(const PrimitiveLayer<PrimitiveT>& layer, Id id);

ConstLanelets findUsagesInLanelets(const LaneletMapLayers& map, const ConstPoint3d& p);
ConstAreas findUsagesInAreas(const LaneletMapLayers& map, const ConstPoint3d& p);

}  // namespace utils

namespace traits {
template <typename T>
struct LayerPrimitive {
  using Type = typename T::PrimitiveT;
};
template <typename T>
struct LayerPrimitive<const T> {
  using Type = typename T::ConstPrimitiveT;
};

template <typename T>
using LayerPrimitiveType = typename LayerPrimitive<T>::Type;
}  // namespace traits

namespace geometry {
/**
 * @brief returns the nearest n primitives to a point.
 * @return vector of the n closest primitives together with their distance in 2D space in
 * ascending order.
 * @see findWithin2d, findWithin3d
 *
 * Other than than LaneletLayer::nearest, this returns the actually closest
 * primitives, not only the closest bounding boxes.
 * This comes at a slightly higher cost, because more primitives from the R-Tree
 * need to be checked.
 *
 * Example: `std::vector<std::pair<double, Lanelet>> closeLanelets = findNearest(laneletMap.laneletLayer,
 * BasicPoint2d(1,0), 5);`
 */
template <typename PrimT>
std::vector<std::pair<double, PrimT>> findNearest(PrimitiveLayer<PrimT>& map, const BasicPoint2d& pt, unsigned count);
template <typename PrimT>
std::vector<std::pair<double, traits::ConstPrimitiveType<PrimT>>> findNearest(const PrimitiveLayer<PrimT>& map,
                                                                              const BasicPoint2d& pt, unsigned count);

#ifndef LANELET_LAYER_DEFINITION
// clang-format off
// NOLINTNEXTLINE
#define EXTERN_FIND_NEAREST(PRIM) extern template std::vector<std::pair<double, PRIM>> findNearest(PrimitiveLayer<PRIM>&, const BasicPoint2d&, unsigned)
// NOLINTNEXTLINE
#define EXTERN_CONST_FIND_NEAREST(PRIM) extern template std::vector<std::pair<double, traits::ConstPrimitiveType<PRIM>>> findNearest(const PrimitiveLayer<PRIM>&, const BasicPoint2d&, unsigned)
// clang-format on
EXTERN_FIND_NEAREST(Area);
EXTERN_FIND_NEAREST(Polygon3d);
EXTERN_FIND_NEAREST(Lanelet);
EXTERN_FIND_NEAREST(LineString3d);
EXTERN_FIND_NEAREST(Point3d);
EXTERN_FIND_NEAREST(RegulatoryElementPtr);
EXTERN_CONST_FIND_NEAREST(Area);
EXTERN_CONST_FIND_NEAREST(Polygon3d);
EXTERN_CONST_FIND_NEAREST(Lanelet);
EXTERN_CONST_FIND_NEAREST(LineString3d);
EXTERN_CONST_FIND_NEAREST(Point3d);
EXTERN_CONST_FIND_NEAREST(RegulatoryElementPtr);
#undef EXTERN_FIND_NEAREST
#undef EXTERN_CONST_FIND_NEAREST
#endif
}  // namespace geometry

}  // namespace lanelet
