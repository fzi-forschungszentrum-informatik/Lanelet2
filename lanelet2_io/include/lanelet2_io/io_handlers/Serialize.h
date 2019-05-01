#pragma once
#include <lanelet2_core/LaneletMap.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/vector.hpp>

namespace boost {
namespace serialization {
template <typename Archive>
void load(Archive& ar, lanelet::Attribute& p, unsigned int /*version*/) {
  std::string val;
  ar& val;
  p = std::move(val);
}

template <typename Archive>
void save(Archive& ar, const lanelet::Attribute& p, unsigned int /*version*/) {
  ar& p.value();
}

template <typename Archive>
void serialize(Archive& ar, std::pair<std::string, lanelet::Attribute>& p, unsigned int /*version*/) {
  ar& p.first;
  ar& p.second;
}

template <typename Archive>
void serialize(Archive& ar, std::pair<std::string, lanelet::RuleParameters>& p, unsigned int /*version*/) {
  ar& p.first;
  ar& p.second;
}

template <typename Archive>
void load(Archive& ar, lanelet::AttributeMap& p, unsigned int /*version*/) {
  size_t size;
  ar& size;
  for (auto i = 0u; i < size; ++i) {
    std::pair<std::string, lanelet::Attribute> val;
    ar& val;
    p.insert(std::move(val));
  }
}

template <typename Archive>
void save(Archive& ar, const lanelet::AttributeMap& p, unsigned int /*version*/) {
  auto size = p.size();
  ar& size;
  for (auto& elem : p) {
    std::pair<std::string, lanelet::Attribute> val = elem;
    ar& val;
  }
}

template <typename Archive>
void load(Archive& ar, lanelet::RuleParameterMap& p, unsigned int /*version*/) {
  size_t size;
  ar& size;
  for (auto i = 0u; i < size; ++i) {
    std::pair<std::string, lanelet::RuleParameters> val;
    ar& val;
    p.insert(std::move(val));
  }
}

template <typename Archive>
void save(Archive& ar, const lanelet::RuleParameterMap& p, unsigned int /*version*/) {
  size_t size = p.size();
  ar& size;
  for (auto& elem : p) {
    std::pair<std::string, lanelet::RuleParameters> val = elem;
    ar& val;
  }
}

//! lanelet +data
template <typename Archive>
void serialize(Archive& /*ar*/, lanelet::LaneletData& /*p*/, unsigned int /*version*/) {}
template <class Archive>
// NOLINTNEXTLINE
inline void save_construct_data(Archive& ar, const lanelet::LaneletData* llt, unsigned int /*version*/) {
  auto lltnc = const_cast<lanelet::LaneletData*>(llt);  // NOLINT
  auto regelems = lltnc->regulatoryElements();
  ar << lltnc->id << lltnc->attributes << lltnc->leftBound() << lltnc->rightBound() << regelems;
  auto hasCenterline = llt->hasCustomCenterline();
  ar << hasCenterline;
  if (hasCenterline) {
    auto centerline = llt->centerline();
    lanelet::LineString3d center(std::const_pointer_cast<lanelet::LineStringData>(centerline.constData()),
                                 centerline.inverted());
    ar << center;
  }
}

template <class Archive>
// NOLINTNEXTLINE
inline void load_construct_data(Archive& ar, lanelet::LaneletData* llt, unsigned int /*version*/
) {
  using namespace lanelet;
  Id id;
  AttributeMap attrs;
  LineString3d left, right;
  RegulatoryElementPtrs regelems;
  ar >> id >> attrs >> left >> right >> regelems;
  new (llt) LaneletData(id, left, right, attrs, regelems);
  bool hasCenterline;
  ar >> hasCenterline;
  if (hasCenterline) {
    LineString3d centerline;
    ar >> centerline;
    llt->setCenterline(centerline);
  }
}

template <typename Archive>
void save(Archive& ar, const lanelet::Lanelet& l, unsigned int /*version*/) {
  auto inv = l.inverted();
  auto ptr = lanelet::utils::removeConst(l.constData());
  ar << inv << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::Lanelet& l, unsigned int /*version*/) {
  std::shared_ptr<lanelet::LaneletData> ptr;
  bool inv;
  ar >> inv >> ptr;
  l = lanelet::Lanelet(ptr, inv);
}
template <typename Archive>
void save(Archive& ar, const lanelet::ConstLanelet& l, unsigned int /*version*/) {
  auto inv = l.inverted();
  auto ptr = lanelet::utils::removeConst(l.constData());
  ar << inv << ptr;
}

template <typename Archive>
void load(Archive& ar, lanelet::ConstLanelet& l, unsigned int /*version*/) {
  std::shared_ptr<lanelet::LaneletData> ptr;
  bool inv;
  ar >> inv >> ptr;
  l = lanelet::Lanelet(ptr, inv);
}

template <typename Archive>
void save(Archive& ar, const lanelet::WeakLanelet& l, unsigned int /*version*/) {
  if (l.expired()) {
    throw lanelet::LaneletError("Can not serialize expired weak pointer!");
  }
  auto sp = l.lock();
  ar& sp;
}
template <typename Archive>
void load(Archive& ar, lanelet::WeakLanelet& l, unsigned int /*version*/) {
  lanelet::Lanelet lanelet;
  ar& lanelet;
  l = lanelet;
}

//! linestring + data
template <typename Archive>
void serialize(Archive& /*ar*/, lanelet::LineStringData& /*l*/, unsigned int /*version*/) {}
template <class Archive>
// NOLINTNEXTLINE
inline void save_construct_data(Archive& ar, const lanelet::LineStringData* l, unsigned int /*version*/) {
  auto lnc = const_cast<lanelet::LineStringData*>(l);  // NOLINT
  ar << lnc->id << lnc->attributes << lnc->points();
}

template <class Archive>
// NOLINTNEXTLINE
inline void load_construct_data(Archive& ar, lanelet::LineStringData* l, unsigned int /*version*/) {
  using namespace lanelet;
  Id id;
  AttributeMap attr;
  Points3d points;
  ar >> id >> attr >> points;
  new (l) LineStringData(id, points, attr);
}

template <typename Archive>
void save(Archive& ar, const lanelet::LineString3d& l, unsigned int /*version*/) {
  auto inv = l.inverted();
  auto ptr = lanelet::utils::removeConst(l.constData());
  ar << inv << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::LineString3d& l, unsigned int /*version*/) {
  std::shared_ptr<lanelet::LineStringData> ptr;
  bool inv{};
  ar >> inv;
  ar >> ptr;
  l = lanelet::LineString3d(ptr, inv);
}
template <typename Archive>
void save(Archive& ar, const lanelet::Polygon3d& l, unsigned int /*version*/) {
  auto inv = l.inverted();
  auto ptr = lanelet::utils::removeConst(l.constData());
  ar << inv << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::Polygon3d& l, unsigned int /*version*/) {
  std::shared_ptr<lanelet::LineStringData> ptr;
  bool inv{};
  ar >> inv;
  ar >> ptr;
  l = lanelet::Polygon3d(ptr, inv);
}
template <typename Archive>
void save(Archive& ar, const lanelet::ConstLineString3d& l, unsigned int /*version*/) {
  auto inv = l.inverted();
  auto ptr = lanelet::utils::removeConst(l.constData());
  ar << inv;
  ar << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::ConstLineString3d& l, unsigned int /*version*/) {
  std::shared_ptr<lanelet::LineStringData> ptr;
  bool inv{};
  ar >> inv;
  ar >> ptr;
  l = lanelet::LineString3d(ptr, inv);
}

template <typename Archive>
void serialize(Archive& ar, lanelet::LineString2d& ls, unsigned int /*version*/) {
  auto ls3d = lanelet::traits::to3D(ls);
  ar& ls3d;
  ls = lanelet::traits::to2D(ls3d);
}

//! point + data
template <typename Archive>
void serialize(Archive& /*ar*/, lanelet::PointData& /*l*/, unsigned int /*version*/) {}
template <class Archive>
// NOLINTNEXTLINE
inline void save_construct_data(Archive& ar, const lanelet::PointData* p, unsigned int /*version*/) {
  ar << p->id << p->attributes << p->point.x() << p->point.y() << p->point.z();
}

template <class Archive>
// NOLINTNEXTLINE
inline void load_construct_data(Archive& ar, lanelet::PointData* p, unsigned int /*version*/) {
  using namespace lanelet;
  Id id;
  AttributeMap attrs;
  BasicPoint3d pt;
  ar >> id >> attrs >> pt.x() >> pt.y() >> pt.z();
  new (p) PointData(id, pt, attrs);
}

template <typename Archive>
void save(Archive& ar, const lanelet::Point3d& p, unsigned int /*version*/) {
  auto ptr = lanelet::utils::removeConst(p.constData());
  ar << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::Point3d& p, unsigned int /*version*/) {
  std::shared_ptr<lanelet::PointData> ptr;
  ar >> ptr;
  p = lanelet::Point3d(ptr);
}
template <typename Archive>
void save(Archive& ar, const lanelet::ConstPoint3d& p, unsigned int /*version*/) {
  ar << lanelet::utils::removeConst(p.constData());
}
template <typename Archive>
void load(Archive& ar, lanelet::ConstPoint3d& p, unsigned int /*version*/) {
  std::shared_ptr<lanelet::PointData> ptr;
  ar >> ptr;
  p = lanelet::Point3d(ptr);
}

//! area
template <typename Archive>
void serialize(Archive& /*ar*/, lanelet::AreaData& /*a*/, unsigned int /*version*/) {}
template <class Archive>
// NOLINTNEXTLINE
inline void save_construct_data(Archive& ar, const lanelet::AreaData* a, unsigned int /*version*/) {
  auto anc = const_cast<lanelet::AreaData*>(a);  // NOLINT
  ar << anc->id;
  ar << anc->attributes;
  ar << anc->innerBounds();
  ar << anc->outerBound();
  ar << anc->regulatoryElements();
}

template <class Archive>
// NOLINTNEXTLINE
inline void load_construct_data(Archive& ar, lanelet::AreaData* a, unsigned int /*version*/) {
  using namespace lanelet;
  Id id;
  AttributeMap attrs;
  InnerBounds inner;
  LineStrings3d outer;
  RegulatoryElementPtrs regelems;

  ar >> id >> attrs >> inner >> outer >> regelems;
  new (a) AreaData(id, outer, inner, attrs, regelems);
}

template <typename Archive>
void save(Archive& ar, const lanelet::Area& a, unsigned int /*version*/) {
  auto ptr = lanelet::utils::removeConst(a.constData());
  ar << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::Area& a, unsigned int /*version*/) {
  std::shared_ptr<lanelet::AreaData> ptr;
  ar >> ptr;
  a = lanelet::Area(ptr);
}
template <typename Archive>
void save(Archive& ar, const lanelet::ConstArea& a, unsigned int /*version*/) {
  auto ptr = lanelet::utils::removeConst(a.constData());
  ar << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::ConstArea& a, unsigned int /*version*/) {
  std::shared_ptr<lanelet::AreaData> ptr;
  ar >> ptr;
  a = lanelet::Area(ptr);
}
template <typename Archive>
void save(Archive& ar, const lanelet::WeakArea& a, unsigned int /*version*/) {
  if (a.expired()) {
    throw lanelet::LaneletError("Can not serialize expired weak pointer!");
  }
  auto sp = a.lock();
  ar& sp;
}
template <typename Archive>
void load(Archive& ar, lanelet::WeakArea& a, unsigned int /*version*/) {
  lanelet::Area area;
  ar& area;
  a = area;
}

//! Regulatory element
template <typename Archive>
void serialize(Archive& /*ar*/, lanelet::RegulatoryElementData& /*r*/, unsigned int /*version*/) {}
template <class Archive>
// NOLINTNEXTLINE
inline void save_construct_data(Archive& ar, const lanelet::RegulatoryElementData* r, unsigned int /*version*/) {
  ar << r->id;
  ar << r->attributes;
  ar << r->parameters;
}

template <class Archive>
// NOLINTNEXTLINE
inline void load_construct_data(Archive& ar, lanelet::RegulatoryElementData* r, unsigned int /*version*/) {
  using namespace lanelet;
  Id id;
  AttributeMap attr;
  RuleParameterMap params;
  ar >> id >> attr >> params;
  new (r) RegulatoryElementData(id, params, attr);
}

template <typename Archive>
void save(Archive& ar, const lanelet::RegulatoryElementPtr& r, unsigned int /*version*/) {
  auto ptr = lanelet::utils::removeConst(r->constData());
  ar << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::RegulatoryElementPtr& r, unsigned int /*version*/) {
  std::shared_ptr<lanelet::RegulatoryElementData> ptr;
  ar >> ptr;
  auto type = ptr->attributes.find(lanelet::AttributeName::Subtype);
  auto subtype = type == ptr->attributes.end() ? "" : type->second.value();
  r = lanelet::RegulatoryElementFactory::create(subtype, ptr);
}
template <typename Archive>
void save(Archive& ar, const lanelet::RegulatoryElementConstPtr& r, unsigned int /*version*/) {
  auto ptr = lanelet::utils::removeConst(r->constData());
  ar << ptr;
}
template <typename Archive>
void load(Archive& ar, lanelet::RegulatoryElementConstPtr& r, unsigned int /*version*/) {
  std::shared_ptr<lanelet::RegulatoryElementData> ptr;
  ar >> ptr;
  auto type = ptr->attributes.find(lanelet::AttributeName::Subtype);
  auto subtype = type == ptr->attributes.end() ? "" : type->second.value();
  r = lanelet::RegulatoryElementFactory::create(subtype, ptr);
}

template <typename Archive>
void save(Archive& ar, const lanelet::LaneletMap& m, unsigned int /*version*/) {
  auto& mnc = const_cast<lanelet::LaneletMap&>(m);  // NOLINT
  auto storeLayer = [&ar](auto& layer) {
    size_t size = layer.size();
    ar << size;
    for (auto& pt : layer) {
      ar << pt;
    }
  };
  storeLayer(mnc.pointLayer);
  storeLayer(mnc.lineStringLayer);
  storeLayer(mnc.polygonLayer);
  storeLayer(mnc.areaLayer);
  storeLayer(mnc.laneletLayer);
  storeLayer(mnc.regulatoryElementLayer);
}

template <typename Archive>
void load(Archive& ar, lanelet::LaneletMap& m, unsigned int /*version*/) {
  auto loadLayer = [&ar](auto& layerMap) {
    using Prim = typename std::decay_t<decltype(layerMap)>::mapped_type;
    size_t size;
    ar >> size;
    for (auto i = 0u; i < size; ++i) {
      Prim p;
      ar >> p;
      layerMap.insert(std::pair<const lanelet::Id, Prim>(lanelet::utils::getId(p), p));
    }
  };
  lanelet::PointLayer::Map pMap;
  lanelet::LineStringLayer::Map lsMap;
  lanelet::PolygonLayer::Map polyMap;
  lanelet::AreaLayer::Map arMap;
  lanelet::LaneletLayer::Map lltMap;
  lanelet::RegulatoryElementLayer::Map reMap;
  loadLayer(pMap);
  loadLayer(lsMap);
  loadLayer(polyMap);
  loadLayer(arMap);
  loadLayer(lltMap);
  loadLayer(reMap);
  m = lanelet::LaneletMap(lltMap, arMap, reMap, polyMap, lsMap, pMap);
}

}  // namespace serialization
}  // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(lanelet::AttributeMap);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::Attribute);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::WeakArea);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::Area);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::WeakLanelet);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::Lanelet);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::Point3d);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::ConstPoint3d);  // NOLINT
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::LineString3d);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::ConstLineString3d);  // NOLINT
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::Polygon3d);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::RuleParameterMap);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::RegulatoryElementPtr);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::RegulatoryElementConstPtr);
BOOST_SERIALIZATION_SPLIT_FREE(lanelet::LaneletMap);
