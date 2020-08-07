#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "lanelet2_python/internal/converter.h"

using namespace boost::python;
using namespace lanelet;

struct AttributeToPythonStr {
  static PyObject* convert(Attribute const& s) { return boost::python::incref(boost::python::object(s.value()).ptr()); }
};

struct AttributeFromPythonStr {
  AttributeFromPythonStr() {
    boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<Attribute>());
  }

  static void* convertible(PyObject* objPtr) {
#if PY_MAJOR_VERSION < 3
    return PyString_Check(objPtr) ? objPtr : nullptr;  // NOLINT
#else
    return PyUnicode_Check(objPtr) ? objPtr : nullptr;  // NOLINT
#endif
  }

  static void construct(PyObject* objPtr, boost::python::converter::rvalue_from_python_stage1_data* data) {
#if PY_MAJOR_VERSION < 3
    const char* value = PyString_AsString(objPtr);
#else
    auto pyStr = PyUnicode_AsUTF8String(objPtr);
    const char* value = PyBytes_AsString(pyStr);
#endif
    if (value == nullptr) {
      boost::python::throw_error_already_set();
    }
    using StorageType = boost::python::converter::rvalue_from_python_storage<Attribute>;
    void* storage = reinterpret_cast<StorageType*>(data)->storage.bytes;  // NOLINT
    new (storage) Attribute(value);
    data->convertible = storage;
  }
};

struct DictToAttributeMapConverter {
  DictToAttributeMapConverter() { converter::registry::push_back(&convertible, &construct, type_id<AttributeMap>()); }
  static void* convertible(PyObject* obj) {
    if (!PyDict_CheckExact(obj)) {  // NOLINT
      return nullptr;
    }
    return obj;
  }
  static void construct(PyObject* obj, converter::rvalue_from_python_stage1_data* data) {
    dict d(borrowed(obj));
    list keys = d.keys();
    list values = d.values();
    AttributeMap attributes;
    for (auto i = 0u; i < len(keys); ++i) {
      std::string key = extract<std::string>(keys[i]);
      std::string value = extract<std::string>(values[i]);
      attributes.insert(std::make_pair(key, value));
    }
    using StorageType = converter::rvalue_from_python_storage<AttributeMap>;
    void* storage = reinterpret_cast<StorageType*>(data)->storage.bytes;  // NOLINT
    new (storage) AttributeMap(attributes);
    data->convertible = storage;
  }
};

struct LineStringOrPolygonToObject {
  static PyObject* convert(const lanelet::LineStringOrPolygon3d& v) {
    if (v.isPolygon()) {
      return incref(object(*v.polygon()).ptr());
    }
    if (v.isLineString()) {
      return incref(object(*v.lineString()).ptr());
    }
    return incref(object().ptr());
  }
};

struct ConstLineStringOrPolygonToObject {
  static PyObject* convert(const lanelet::ConstLineStringOrPolygon3d& v) {
    if (v.isPolygon()) {
      return incref(object(*v.polygon()).ptr());
    }
    if (v.isLineString()) {
      return incref(object(*v.lineString()).ptr());
    }
    return incref(object().ptr());
  }
};

struct ConstLaneletOrAreaToObject {
  static PyObject* convert(const lanelet::ConstLaneletOrArea& v) {
    if (v.isArea()) {
      return incref(object(*v.area()).ptr());
    }
    if (v.isLanelet()) {
      return incref(object(*v.lanelet()).ptr());
    }
    return incref(object().ptr());
  }
};

template <class T>
struct MapItem {
  using K = typename T::key_type;
  using V = typename T::mapped_type;
  using Iter = typename T::const_iterator;

  MapItem& fromPython() {
    &MapItem::convertible, &MapItem::init, boost::python::type_id<T>();
    return *this;
  }
  static list keys(T const& x) {
    list t;
    for (auto it = x.begin(); it != x.end(); ++it) {
      t.append(it->first);
    }
    return t;
  }
  static list values(T const& x) {
    list t;
    for (auto it = x.begin(); it != x.end(); ++it) {
      t.append(it->second);
    }
    return t;
  }
  static list items(T const& x) {
    list t;
    for (auto it = x.begin(); it != x.end(); ++it) {
      t.append(boost::python::make_tuple(it->first, it->second));
    }
    return t;
  }
  static void* convertible(PyObject* object) { return PyObject_GetIter(object) != nullptr ? object : nullptr; }
  static std::shared_ptr<T> init(boost::python::dict& pyDict) {
    auto mapPtr = std::make_shared<T>();
    auto& map = *mapPtr;
    boost::python::list keys = pyDict.keys();
    for (int i = 0; i < len(keys); ++i) {  // NOLINT
      boost::python::extract<K> extractedKey(keys[i]);
      if (!extractedKey.check()) {
        PyErr_SetString(PyExc_KeyError, "Key invalid!");
        throw_error_already_set();
      }
      K key = extractedKey;
      boost::python::extract<V> extractedVal(pyDict[key]);
      if (!extractedVal.check()) {
        PyErr_SetString(PyExc_KeyError, "Value invalid!");
        throw_error_already_set();
      }
      V value = extractedVal;
      map[key] = value;  // NOLINT
    }
    return mapPtr;
  }
};

template <typename T>
void setXWrapper(T& obj, double x) {
  obj.x() = x;
}
template <typename T>
void setYWrapper(T& obj, double y) {
  obj.y() = y;
}

template <typename T>
void setZWrapper(T& obj, double z) {
  obj.z() = z;
}
template <typename T>
double getXWrapper(const T& obj) {
  return obj.x();
}
template <typename T>
double getYWrapper(const T& obj) {
  return obj.y();
}

template <typename T>
double getZWrapper(const T& obj) {
  return obj.z();
}

template <typename Func>
auto getRefFunc(Func&& f) {
  return make_function(std::forward<Func>(f), return_internal_reference<>());
}

template <typename T>
void setAttributeWrapper(T& obj, const AttributeMap& attr) {
  obj.attributes() = attr;
}

template <typename PrimT>
class IsPrimitive : public def_visitor<IsPrimitive<PrimT>> {
  using ConstT = const PrimT;

 public:
  template <typename ClassT>
  void visit(ClassT& c) const {
    const AttributeMap& (PrimT::*attr)() const = &PrimT::attributes;
    c.add_property("id", &PrimT::id, &PrimT::setId);
    c.add_property("attributes", getRefFunc(attr), setAttributeWrapper<PrimT>);
    c.def(self == self);  // NOLINT
    c.def(self != self);  // NOLINT
    c.def(self_ns::str(self_ns::self));
    c.def(
        "__hash__", +[](const PrimT& self) { return std::hash<PrimT>()(self); });
  }
};

template <typename PrimT>
class IsConstPrimitive : public def_visitor<IsConstPrimitive<PrimT>> {
  friend class def_visitor_access;

 public:
  template <typename ClassT>
  void visit(ClassT& c) const {
    c.add_property("id", &PrimT::id);
    const AttributeMap& (PrimT::*attr)() const = &PrimT::attributes;
    c.add_property("attributes", getRefFunc(attr));
    c.def(self == self);  // NOLINT
    c.def(self != self);  // NOLINT
    c.def(self_ns::str(self_ns::self));
    c.def(
        "__hash__", +[](const PrimT& self) { return std::hash<PrimT>()(self); });
  }
};

template <typename LsT, bool InternalRef = true>
class IsConstLineString : public def_visitor<IsConstLineString<LsT, InternalRef>> {
  friend class def_visitor_access;

 public:
  template <typename ClassT>
  void visit(ClassT& c) const {
    c.def("__iter__", iterator<LsT>()).def("__len__", &LsT::size).def("inverted", &LsT::inverted);
    addGetitem<InternalRef>(c);
  }
  template <bool InternalRefVal, typename ClassT>
  std::enable_if_t<InternalRefVal> addGetitem(ClassT& c) const {
    c.def("__getitem__", wrappers::getItem<LsT>, return_internal_reference<>());
  }
  template <bool InternalRefVal, typename ClassT>
  std::enable_if_t<!InternalRefVal> addGetitem(ClassT& c) const {
    c.def("__getitem__", wrappers::getItem<LsT>, return_value_policy<return_by_value>());
  }

 private:
  bool internalRef_{false};
};

template <typename LsT>
class IsLineString : public def_visitor<IsLineString<LsT>> {
  friend class def_visitor_access;

 public:
  template <typename ClassT>
  void visit(ClassT& c) const {
    c.def("__setitem__", wrappers::setItem<LsT, typename LsT::PointType>)
        .def("__delitem__", wrappers::delItem<LsT>)
        .def("append", &LsT::push_back)
        .def("__iter__", iterator<LsT>())
        .def("__len__", &LsT::size)
        .def("inverted", &LsT::inverted);
    addGetitem(c);
  }
  template <typename ClassT>
  void addGetitem(ClassT& c) const {
    c.def("__getitem__", wrappers::getItem<LsT>, return_internal_reference<>());
  }
};

template <typename T>
class IsHybridMap : public def_visitor<IsHybridMap<T>> {
  friend class def_visitor_access;

 public:
  template <typename ClassT>
  void visit(ClassT& c) const {
    c.def("__init__", make_constructor(MapItem<T>::init))
        .def(map_indexing_suite<T, true>())
        .def("keys", MapItem<T>::keys)
        .def("values", MapItem<T>::values)
        .def("items", MapItem<T>::items,
             "Iterates over the key-value pairs")
        .def(self == self)   // NOLINT
        .def(self != self);  // NOLINT
  }
};

template <typename T>
T addWrapper(const T& rhs, const T& lhs) {
  return (rhs + lhs);
}

template <typename T>
T subWrapper(const T& rhs, const T& lhs) {
  return (rhs - lhs);
}

template <typename T1, typename T2>
T1 mulWrapper(const T1& rhs, const T2& lhs) {
  return (rhs * lhs).eval();
}

template <typename T1, typename T2>
T2 rmulWrapper(const T1& rhs, const T2& lhs) {
  return (rhs * lhs).eval();
}

template <typename T1, typename T2>
T1 divWrapper(const T1& rhs, const T2& lhs) {
  return (rhs / lhs).eval();
}

template <typename T>
using SetAttrSig = void (T::*)(const std::string&, const Attribute&);

template <typename T>
using GetAttrSig = const Attribute& (ConstPrimitive<T>::*)(const std::string&) const;

template <typename LayerT = PointLayer, typename... ClassArgs>
auto wrapLayer(const char* layerName) {
  auto get = static_cast<typename LayerT::PrimitiveT (LayerT::*)(Id)>(&LayerT::get);
  auto search = static_cast<typename LayerT::PrimitiveVec (LayerT::*)(const BoundingBox2d&)>(&LayerT::search);
  auto nearest =
      static_cast<typename LayerT::PrimitiveVec (LayerT::*)(const BasicPoint2d&, unsigned)>(&LayerT::nearest);
  return class_<LayerT, boost::noncopyable, ClassArgs...>(layerName, no_init)
      .def("exists", &LayerT::exists, "Checks if a point exists")
      .def("get", get, "Gets a point with specified Id")
      .def("__iter__", iterator<LayerT>())
      .def("__len__", &LayerT::size)
      .def(
          "__getitem__", +[](LayerT& self, Id idx) { return self.get(idx); })
      .def("search", search, "Search in a search area")
      .def("nearest", nearest, "Get nearest n points")
      .def("uniqueId", &LayerT::uniqueId);
}

template <typename PrimT>
auto selectAdd() {
  return static_cast<void (LaneletMap::*)(PrimT)>(&LaneletMap::add);
}
template <typename PrimT>
auto selectSubmapAdd() {
  return static_cast<void (LaneletSubmap::*)(PrimT)>(&LaneletSubmap::add);
}

template <typename RegelemT>
std::vector<std::shared_ptr<RegelemT>> regelemAs(Lanelet& llt) {
  return llt.regulatoryElementsAs<RegelemT>();
}

template <typename RegelemT>
std::vector<std::shared_ptr<RegelemT>> constRegelemAs(ConstLanelet& llt) {
  return utils::transform(llt.regulatoryElementsAs<RegelemT>(),
                          [](auto& e) { return std::const_pointer_cast<RegelemT>(e); });
}

template <typename PrimT>
LaneletMapPtr createMapWrapper(const PrimT& prim) {
  return utils::createMap(prim);
}

template <typename PrimT>
LaneletSubmapPtr createSubmapWrapper(const PrimT& prim) {
  return utils::createSubmap(prim);
}

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  class_<BasicPoint2d>("BasicPoint2d", "A simple point", init<double, double>((arg("x") = 0., arg("y") = 0.)))
      .def(init<>("BasicPoint2d()"))
      .add_property("x", getXWrapper<BasicPoint2d>, setXWrapper<BasicPoint2d>, "x coordinate")
      .add_property("y", getYWrapper<BasicPoint2d>, setYWrapper<BasicPoint2d>, "y coordinate")
      .def("__add__", addWrapper<BasicPoint2d>)
      .def("__sub__", subWrapper<BasicPoint2d>)
      .def("__mul__", mulWrapper<BasicPoint2d, double>)
      .def("__rmul__", mulWrapper<BasicPoint2d, double>)
      .def("__div__", divWrapper<BasicPoint2d, double>)
      .def(self_ns::str(self_ns::self));

  class_<Eigen::Vector2d>("Vector2d", "A simple point", no_init)
      .add_property("x", getXWrapper<BasicPoint2d>, setXWrapper<BasicPoint2d>, "x coordinate")
      .add_property("y", getYWrapper<BasicPoint2d>, setYWrapper<BasicPoint2d>, "y coordinate")
      .def("__add__", addWrapper<BasicPoint2d>)
      .def("__sub__", subWrapper<BasicPoint2d>)
      .def("__mul__", mulWrapper<BasicPoint2d, double>)
      .def("__rmul__", mulWrapper<BasicPoint2d, double>)
      .def("__div__", divWrapper<BasicPoint2d, double>)
      .def(self_ns::str(self_ns::self));

  implicitly_convertible<Eigen::Vector2d, BasicPoint2d>();

  class_<BasicPoint3d>("BasicPoint3d", "A simple point",
                       init<double, double, double>((arg("x") = 0., arg("y") = 0., arg("z") = 0.)))
      .add_property("x", getXWrapper<BasicPoint3d>, setXWrapper<BasicPoint3d>, "x coordinate")
      .add_property("y", getYWrapper<BasicPoint3d>, setYWrapper<BasicPoint3d>, "y coordinate")
      .add_property("z", getZWrapper<BasicPoint3d>, setZWrapper<BasicPoint3d>, "z coordinate")
      .def("__add__", addWrapper<BasicPoint3d>)
      .def("__sub__", subWrapper<BasicPoint3d>)
      .def("__mul__", mulWrapper<BasicPoint3d, double>)
      .def("__rmul__", mulWrapper<BasicPoint3d, double>)
      .def("__div__", divWrapper<BasicPoint3d, double>)
      .def(self_ns::str(self_ns::self));

  class_<BoundingBox2d>("BoundingBox2d", init<BasicPoint2d, BasicPoint2d>("BoundingBox2d(minPoint, maxPoint"))
      .add_property(
          "min", +[](BoundingBox2d& self) { return self.min(); })
      .add_property(
          "max", +[](BoundingBox2d& self) { return self.max(); });

  class_<BoundingBox3d>("BoundingBox3d", init<BasicPoint3d, BasicPoint3d>("BoundingBox3d(minPoint, maxPoint"))
      .add_property(
          "min", +[](BoundingBox3d& self) { return self.min(); })
      .add_property(
          "max", +[](BoundingBox3d& self) { return self.max(); });

  boost::python::to_python_converter<Attribute, AttributeToPythonStr>();

  using ::converters::IterableConverter;
  using ::converters::OptionalConverter;
  using ::converters::PairConverter;
  using ::converters::ToOptionalConverter;
  using ::converters::VariantConverter;
  using ::converters::VectorToListConverter;
  using ::converters::WeakConverter;

  VariantConverter<RuleParameter>();
  VariantConverter<ConstRuleParameter>();

  VectorToListConverter<Points3d>();
  VectorToListConverter<Points2d>();
  VectorToListConverter<BasicPoints3d>();
  VectorToListConverter<std::vector<BasicPoint2d>>();
  VectorToListConverter<ConstPoints3d>();
  VectorToListConverter<ConstPoints2d>();
  VectorToListConverter<LineStrings3d>();
  VectorToListConverter<LineStrings2d>();
  VectorToListConverter<ConstLineStrings3d>();
  VectorToListConverter<ConstLineStrings2d>();
  VectorToListConverter<BasicPolygon3d>();
  VectorToListConverter<BasicPolygon2d>();
  VectorToListConverter<Polygons3d>();
  VectorToListConverter<Polygons2d>();
  VectorToListConverter<ConstPolygons3d>();
  VectorToListConverter<ConstPolygons2d>();
  VectorToListConverter<CompoundPolygons3d>();
  VectorToListConverter<CompoundPolygons2d>();
  VectorToListConverter<Lanelets>();
  VectorToListConverter<ConstLanelets>();
  VectorToListConverter<LaneletSequences>();
  VectorToListConverter<LaneletsWithStopLines>();
  VectorToListConverter<RuleParameters>();
  VectorToListConverter<ConstRuleParameters>();
  VectorToListConverter<RegulatoryElementPtrs>();
  VectorToListConverter<RegulatoryElementConstPtrs>();
  VectorToListConverter<LineStringsOrPolygons3d>();
  VectorToListConverter<ConstLineStringsOrPolygons3d>();
  VectorToListConverter<ConstLaneletOrAreas>();
  VectorToListConverter<Areas>();
  VectorToListConverter<std::vector<TrafficLight::Ptr>>();
  VectorToListConverter<std::vector<TrafficSign::Ptr>>();
  VectorToListConverter<std::vector<SpeedLimit::Ptr>>();
  VectorToListConverter<std::vector<RightOfWay::Ptr>>();
  VectorToListConverter<std::vector<AllWayStop::Ptr>>();
  VectorToListConverter<std::vector<std::shared_ptr<const TrafficLight>>>();
  VectorToListConverter<std::vector<std::shared_ptr<const TrafficSign>>>();
  VectorToListConverter<std::vector<std::shared_ptr<const SpeedLimit>>>();
  VectorToListConverter<std::vector<std::shared_ptr<const RightOfWay>>>();
  VectorToListConverter<std::vector<std::shared_ptr<const AllWayStop>>>();
  VectorToListConverter<std::vector<std::string>>();
  AttributeFromPythonStr();
  DictToAttributeMapConverter();

  OptionalConverter<Point2d>();
  OptionalConverter<Point3d>();
  OptionalConverter<ConstPoint2d>();
  OptionalConverter<ConstPoint3d>();
  OptionalConverter<LineString3d>();
  OptionalConverter<LineString2d>();
  OptionalConverter<ConstLineString3d>();
  OptionalConverter<ConstLineString2d>();
  OptionalConverter<LineStrings3d>();
  OptionalConverter<LineStrings2d>();
  OptionalConverter<ConstLineStrings3d>();
  OptionalConverter<ConstLineStrings2d>();
  OptionalConverter<LineStringsOrPolygons3d>();
  OptionalConverter<Lanelet>();
  OptionalConverter<ConstLanelet>();
  OptionalConverter<LaneletSequence>();
  OptionalConverter<Area>();
  OptionalConverter<ConstArea>();
  OptionalConverter<Polygon2d>();
  OptionalConverter<Polygon3d>();
  OptionalConverter<ConstPolygon2d>();
  OptionalConverter<ConstPolygon3d>();
  OptionalConverter<RuleParameter>();
  OptionalConverter<ConstRuleParameter>();
  OptionalConverter<RegulatoryElement>();

  PairConverter<std::pair<BasicPoint2d, BasicPoint2d>>();
  PairConverter<std::pair<BasicPoint3d, BasicPoint3d>>();

  WeakConverter<WeakLanelet>();
  WeakConverter<WeakArea>();

  // Register interable conversions.
  IterableConverter()
      .fromPython<Points3d>()
      .fromPython<Points2d>()
      .fromPython<ConstLineStrings3d>()
      .fromPython<ConstLineStrings2d>()
      .fromPython<LineStrings3d>()
      .fromPython<LineStrings2d>()
      .fromPython<Polygons2d>()
      .fromPython<Polygons3d>()
      .fromPython<Lanelets>()
      .fromPython<ConstLanelets>()
      .fromPython<LaneletsWithStopLines>()
      .fromPython<Areas>()
      .fromPython<ConstAreas>()
      .fromPython<RegulatoryElementPtrs>()
      .fromPython<LineStringsOrPolygons3d>()
      .fromPython<ConstLineStringsOrPolygons3d>();

  ToOptionalConverter().fromPython<LineString3d>();

  to_python_converter<LineStringOrPolygon3d, LineStringOrPolygonToObject>();
  to_python_converter<ConstLineStringOrPolygon3d, ConstLineStringOrPolygonToObject>();
  to_python_converter<ConstLaneletOrArea, ConstLaneletOrAreaToObject>();
  implicitly_convertible<LineString3d, LineStringOrPolygon3d>();
  implicitly_convertible<Polygon3d, LineStringOrPolygon3d>();
  implicitly_convertible<ConstLineString3d, ConstLineStringOrPolygon3d>();
  implicitly_convertible<ConstPolygon3d, ConstLineStringOrPolygon3d>();
  implicitly_convertible<ConstLanelet, ConstLaneletOrArea>();
  implicitly_convertible<ConstArea, ConstLaneletOrArea>();

  class_<AttributeMap>("AttributeMap", init<>("AttributeMap()"))
      .def(IsHybridMap<AttributeMap>())
      .def(self_ns::str(self_ns::self));

  class_<RuleParameterMap>("RuleParameterMap", init<>("RuleParameterMap()")).def(IsHybridMap<RuleParameterMap>());

  class_<ConstRuleParameterMap>("ConstRuleParameterMap", init<>("ConstRuleParameterMap()"))
      .def(IsHybridMap<ConstRuleParameterMap>());
  class_<ConstPoint2d>("ConstPoint2d", no_init)
      .def(IsConstPrimitive<ConstPoint2d>())
      .add_property("x", getXWrapper<ConstPoint2d>, "x coordinate")
      .add_property("y", getYWrapper<ConstPoint2d>, "y coordinate");

  class_<Point2d, bases<ConstPoint2d>>(
      "Point2d", "Lanelets 2d point primitive",
      init<Id, BasicPoint3d, AttributeMap>((arg("id"), arg("point"), arg("attributes") = AttributeMap())))
      .def(init<>("Point2d()"))
      .def(init<Point3d>("Point3d()"))
      .def(init<Id, double, double, double, AttributeMap>(
          (arg("id"), arg("x"), arg("y"), arg("z") = 0., arg("attributes") = AttributeMap())))
      .add_property("x", getXWrapper<Point2d>, setXWrapper<Point2d>, "x coordinate")
      .add_property("y", getYWrapper<Point2d>, setYWrapper<Point2d>, "y coordinate")
      .def("basicPoint", &ConstPoint2d::basicPoint, return_internal_reference<>())
      .def(IsPrimitive<Point2d>());

  class_<ConstPoint3d>("ConstPoint3d", no_init)
      .def(IsConstPrimitive<ConstPoint3d>())
      .add_property("x", getXWrapper<ConstPoint3d>, "x coordinate")
      .add_property("y", getYWrapper<ConstPoint3d>, "y coordinate")
      .add_property("z", getZWrapper<ConstPoint3d>, "z coordinate")
      .def("basicPoint", &ConstPoint3d::basicPoint, return_internal_reference<>());

  class_<Point3d, bases<ConstPoint3d>>(
      "Point3d", "Lanelets 3d point primitive",
      init<Id, BasicPoint3d, AttributeMap>((arg("id"), arg("point"), arg("attributes") = AttributeMap())))
      .def(init<>("Point3d()"))
      .def(init<Point2d>("Point2d()"))
      .def(init<Id, double, double, double, AttributeMap>(
          (arg("id"), arg("x"), arg("y"), arg("z") = 0., arg("attributes") = AttributeMap())))
      .add_property("x", getXWrapper<Point3d>, setXWrapper<Point3d>, "x coordinate")
      .add_property("y", getYWrapper<Point3d>, setYWrapper<Point3d>, "y coordinate")
      .add_property("z", getZWrapper<Point3d>, setZWrapper<Point3d>, "z coordinate")
      .def(IsPrimitive<Point3d>());

  class_<GPSPoint, std::shared_ptr<GPSPoint>>("GPSPoint", "A raw GPS point", no_init)
      .def("__init__", make_constructor(
                           +[](double lat, double lon, double alt) {
                             return std::make_shared<GPSPoint>(GPSPoint({lat, lon, alt}));
                           },
                           default_call_policies(), (arg("lat") = 0., arg("lon") = 0., arg("alt") = 0)))
      .def_readwrite("lat", &GPSPoint::lat)
      .def_readwrite("lon", &GPSPoint::lon)
      .def_readwrite("alt", &GPSPoint::ele);

  class_<ConstLineString2d>("ConstLineString2d", "Immutable 2d lineString primitive",
                            init<ConstLineString3d>("ConstLineString2d(ConstLineString3d)"))
      .def("invert", &ConstLineString2d::invert)
      .def(IsConstLineString<ConstLineString2d>())
      .def(IsConstPrimitive<ConstLineString2d>());

  class_<LineString2d, bases<ConstLineString2d>>(
      "LineString2d", "Lanelets 2d lineString primitive",
      init<Id, Points3d, AttributeMap>("LineString2d(id, point_list, attributes)"))
      .def(init<Id, Points3d>("LineString2d(id, point_list"))
      .def(init<LineString3d>("LineString2d(LineString3d)"))
      .def("invert", &LineString2d::invert)
      .def(IsLineString<LineString2d>());

  class_<ConstLineString3d>("ConstLineString3d", "Immutable 3d lineString primitive",
                            init<ConstLineString2d>("ConstLineString3d(ConstLineString2d)"))
      .def(init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def("invert", &ConstLineString3d::invert)
      .def(IsConstLineString<ConstLineString3d>())
      .def(IsConstPrimitive<ConstLineString3d>());

  class_<LineString3d, bases<ConstLineString3d>>(
      "LineString3d", "Lanelets 3d lineString primitive",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def(init<LineString2d>("LineString3d(LineString2d)"))
      .def("invert", &LineString3d::invert)
      .def(IsLineString<LineString3d>())
      .def(IsPrimitive<LineString3d>());

  class_<ConstHybridLineString2d>("ConstHybridLineString2d", "A Linestring that behaves like a normal BasicLineString",
                                  init<LineString2d>("ConstHybridLineString2d(LineString2d"))
      .def(init<ConstLineString2d>())
      .def("invert", &ConstHybridLineString2d::invert)
      .def(IsConstLineString<ConstHybridLineString2d, false>())
      .def(IsConstPrimitive<ConstHybridLineString2d>());

  class_<ConstHybridLineString3d>("ConstHybridLineString3d", "A Linestring that behaves like a normal BasicLineString",
                                  init<LineString3d>("ConstHybridLineString3d(LineString3d"))
      .def("invert", &ConstHybridLineString3d::invert)
      .def(IsConstLineString<ConstHybridLineString3d, false>())
      .def(IsConstPrimitive<ConstHybridLineString3d>());

  class_<CompoundLineString2d>("CompoundLineString2d", "A LineString composed of multiple linestrings",
                               init<ConstLineStrings2d>())
      .def(IsConstLineString<CompoundLineString2d>())
      .def("lineStrings", &CompoundLineString2d::lineStrings)
      .def("ids", &CompoundLineString2d::ids, "ids of the linestrings");

  class_<CompoundLineString3d>("CompoundLineString3d", "A LineString composed of multiple linestrings",
                               init<ConstLineStrings3d>())
      .def(IsConstLineString<CompoundLineString3d>())
      .def("lineStrings", &CompoundLineString3d::lineStrings)
      .def("ids", &CompoundLineString3d::ids, "ids of the linestrings");

  class_<ConstPolygon2d>(
      "ConstPolygon2d", "A two-dimensional lanelet polygon",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def(IsConstLineString<ConstPolygon2d>())
      .def(IsConstPrimitive<ConstPolygon2d>());

  class_<ConstPolygon3d>(
      "ConstPolygon3d", "A three-dimensional lanelet polygon",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def(IsConstLineString<ConstPolygon3d>())
      .def(IsConstPrimitive<ConstPolygon3d>());

  class_<Polygon2d, bases<ConstPolygon2d>>("Polygon2d", "A two-dimensional lanelet polygon",
                                           init<Id, Points3d, AttributeMap>("Polygon2d(Id, point_list, attributes"))
      .def(IsLineString<Polygon2d>())
      .def(IsPrimitive<Polygon2d>());

  class_<Polygon3d, bases<ConstPolygon3d>>(
      "Polygon3d", "A three-dimensional lanelet polygon",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def(IsLineString<Polygon3d>())
      .def(IsPrimitive<Polygon3d>());

  class_<CompoundPolygon2d>("CompoundPolygon2d", "A polygon composed of multiple linestrings",
                            init<ConstLineStrings2d>())
      .def(IsConstLineString<CompoundPolygon2d>())
      .def("lineStrings", &CompoundPolygon2d::lineStrings)
      .def("ids", &CompoundPolygon2d::ids, "ids of the linestrings");

  class_<CompoundPolygon3d>("CompoundPolygon3d", "A polygon composed of multiple linestrings",
                            init<ConstLineStrings3d>())
      .def(IsConstLineString<CompoundPolygon3d>())
      .def("lineStrings", &CompoundPolygon3d::lineStrings)
      .def("ids", &CompoundPolygon3d::ids, "ids of the linestrings");

  class_<ConstLanelet>("ConstLanelet", "An immutable lanelet primitive",
                       init<Id, LineString3d, LineString3d, AttributeMap>(
                           (arg("id"), arg("leftBound"), arg("rightBound"), arg("attributes") = AttributeMap())))
      .def(init<Lanelet>())
      .def(IsConstPrimitive<ConstLanelet>())
      .add_property("centerline", &ConstLanelet::centerline, "Centerline of the lanelet")
      .add_property("leftBound", &ConstLanelet::leftBound, "Left boundary of lanelet")
      .add_property("rightBound", &ConstLanelet::rightBound, "Right boundary of lanelet")
      .add_property("regulatoryElements",
                    make_function(&ConstLanelet::regulatoryElements, return_value_policy<return_by_value>()),
                    "Regulatory elements of the lanelet")
      .def("trafficLights", constRegelemAs<TrafficLight>, "traffic light regulatory elements")
      .def("trafficSigns", constRegelemAs<TrafficSign>, "traffic sign regulatory elements")
      .def("speedLimits", constRegelemAs<SpeedLimit>, "speed limit regulatory elements")
      .def("rightOfWay", constRegelemAs<RightOfWay>, "right of way regulatory elements")
      .def("allWayStop", constRegelemAs<AllWayStop>, "all way stop regulatory elements")
      .def("invert", &ConstLanelet::invert, "Returns inverted lanelet (flipped left/right bound, etc")
      .def("inverted", &ConstLanelet::inverted, "Returns whether this lanelet has been inverted")
      .def("polygon2d", &ConstLanelet::polygon2d, "Outline of this lanelet as 2d polygon")
      .def("polygon3d", &ConstLanelet::polygon3d, "Outline of this lanelet as 3d polygon")
      .def("resetCache", &ConstLanelet::resetCache,
           "Reset the cache. Forces update of the centerline if points have chagned");

  auto left = static_cast<LineString3d (Lanelet::*)()>(&Lanelet::leftBound);
  auto right = static_cast<LineString3d (Lanelet::*)()>(&Lanelet::rightBound);
  auto regelems = static_cast<const RegulatoryElementPtrs& (Lanelet::*)()>(&Lanelet::regulatoryElements);

  class_<Lanelet, bases<ConstLanelet>>(
      "Lanelet", "The famous lanelet primitive",
      init<Id, LineString3d, LineString3d, AttributeMap>(
          (arg("id"), arg("leftBound"), arg("rightBound"), arg("attributes") = AttributeMap())))
      .def(IsPrimitive<Lanelet>())
      .add_property("centerline", &Lanelet::centerline, &Lanelet::setCenterline, "Centerline of the lanelet")
      .add_property("leftBound", left, &Lanelet::setLeftBound, "Left boundary of lanelet")
      .add_property("rightBound", right, &Lanelet::setRightBound, "Right boundary of lanelet")
      .add_property("regulatoryElements", make_function(regelems, return_value_policy<return_by_value>()),
                    "Regulatory elements of the lanelet")
      .def("trafficLights", regelemAs<TrafficLight>, "traffic light regulatory elements")
      .def("trafficSigns", regelemAs<TrafficSign>, "traffic sign regulatory elements")
      .def("speedLimits", regelemAs<SpeedLimit>, "speed limit regulatory elements")
      .def("rightOfWay", regelemAs<RightOfWay>, "right of way regulatory elements")
      .def("allWayStop", regelemAs<AllWayStop>, "all way stop regulatory elements")
      .def("addRegulatoryElement", &Lanelet::addRegulatoryElement)
      .def("removeRegulatoryElement", &Lanelet::removeRegulatoryElement)
      .def("invert", &Lanelet::invert, "Returns inverted lanelet (flipped left/right bound, etc")
      .def("inverted", &ConstLanelet::inverted, "Returns whether this lanelet has been inverted")
      .def("polygon2d", &ConstLanelet::polygon2d, "Outline of this lanelet as 2d polygon")
      .def("polygon3d", &ConstLanelet::polygon3d, "Outline of this lanelet as 3d polygon");

  class_<LaneletSequence>("LaneletSequence", "A combined lane formed from multiple lanelets", init<ConstLanelets>())
      .add_property("centerline", &LaneletSequence::centerline, "Centerline of the lanelet")
      .add_property("leftBound", &LaneletSequence::leftBound, "Left boundary of lanelet")
      .add_property("rightBound", &LaneletSequence::rightBound, "Right boundary of lanelet")
      .def("invert", &LaneletSequence::invert, "Returns inverted lanelet (flipped left/right bound, etc")
      .def("inverted", &LaneletSequence::inverted, "Returns whether this lanelet has been inverted")
      .def("polygon2d", &LaneletSequence::polygon2d, "Outline of this lanelet as 2d polygon")
      .def("polygon3d", &LaneletSequence::polygon3d, "Outline of this lanelet as 3d polygon")
      .def("lanelets", &LaneletSequence::lanelets, "Lanelets that make up this compound lanelet")
      .def("__iter__", iterator<LaneletSequence>())
      .def("__len__", &LaneletSequence::size)
      .def("inverted", &LaneletSequence::inverted)
      .def("__getitem__", wrappers::getItem<LaneletSequence>, return_internal_reference<>());

  class_<ConstLaneletWithStopLine>("ConstLaneletWithStopLine", "A lanelet with a stopline", no_init)
      .add_property("lanelet", &ConstLaneletWithStopLine::lanelet)
      .add_property("stopLine", &ConstLaneletWithStopLine::stopLine);

  class_<LaneletWithStopLine>("LaneletWithStopLine", "A lanelet with a stopline", no_init)
      .add_property("lanelet", &LaneletWithStopLine::lanelet)
      .add_property("stopLine", &LaneletWithStopLine::stopLine);

  class_<ConstArea>("ConstArea", "Represents an area, potentially with holes, in the map",
                    boost::python::init<Id, LineStrings3d, InnerBounds, AttributeMap>(
                        "Lanelet(id, outerBound, innerBounds, attributes"))
      .def(init<Id, LineStrings3d, InnerBounds>("Lanelet(id, outerBound, innerBounds"))
      .def(init<Id, LineStrings3d>("Lanelet(id, outerBound"))
      .def(IsConstPrimitive<ConstArea>())
      .add_property("outerBound", &ConstArea::outerBound)
      .add_property("innerBounds", &ConstArea::innerBounds)
      .def("outerBoundPolygon", &ConstArea::outerBoundPolygon)
      .def("innerBoundPolygon", &ConstArea::innerBoundPolygons);

  auto outerBound = static_cast<const LineStrings3d& (Area::*)()>(&Area::outerBound);
  auto innerBounds = static_cast<const std::vector<LineStrings3d>& (Area::*)()>(&Area::innerBounds);
  class_<Area, bases<ConstArea>>("Area", "Represents an area, potentially with holes, in the map",
                                 boost::python::init<Id, LineStrings3d, InnerBounds, AttributeMap>(
                                     "Lanelet(id, outerBound, innerBounds, attributes"))
      .def(init<Id, LineStrings3d, InnerBounds>("Lanelet(id, outerBound, innerBounds"))
      .def(init<Id, LineStrings3d>("Lanelet(id, outerBound"))
      .def(IsPrimitive<Area>())
      .add_property("outerBound", getRefFunc(outerBound), &Area::setOuterBound)
      .add_property("innerBounds", getRefFunc(innerBounds), &Area::setInnerBounds)
      .add_property(
          "regulatoryElements", +[](Area& self) { return self.regulatoryElements(); },
          "Regulatory elements of the area")
      .def("addRegulatoryElement", &Area::addRegulatoryElement)
      .def("removeRegulatoryElement", &Area::removeRegulatoryElement)
      .def("outerBoundPolygon", &Area::outerBoundPolygon)
      .def("innerBoundPolygon", &Area::innerBoundPolygons);

  using GetParamSig = ConstRuleParameterMap (RegulatoryElement::*)() const;

  class_<RegulatoryElement, boost::noncopyable, RegulatoryElementPtr>(
      "RegulatoryElement", "A Regulatory element defines traffic rules that affect a lanelet", no_init)
      .def(IsConstPrimitive<RegulatoryElement>())
      .add_property("id", &RegulatoryElement::id, &RegulatoryElement::setId)
      .add_property("parameters", static_cast<GetParamSig>(&RegulatoryElement::getParameters),
                    "the parameters (ie traffic signs, lanelets) that affect "
                    "this RegulatoryElement")
      .add_property("roles", &RegulatoryElement::roles)
      .def("find", &RegulatoryElement::find<ConstRuleParameter>, "Returns a primitive with matching id, else None")
      .def("__len__", &RegulatoryElement::size);
  register_ptr_to_python<RegulatoryElementConstPtr>();

  class_<TrafficLight, boost::noncopyable, std::shared_ptr<TrafficLight>, bases<RegulatoryElement>>(
      "TrafficLight", "A traffic light regulatory element", no_init)
      .def("__init__", make_constructor(&TrafficLight::make, default_call_policies(),
                                        (arg("id"), arg("attributes"), arg("trafficLights"),
                                         arg("stopLine") = Optional<LineString3d>())))
      .add_property(
          "stopLine", +[](TrafficLight& self) { return self.stopLine(); }, &TrafficLight::setStopLine)
      .add_property(
          "trafficLights", +[](TrafficLight& self) { return self.trafficLights(); })
      .def("addTrafficLight", &TrafficLight::addTrafficLight)
      .def("removeTrafficLight", &TrafficLight::removeTrafficLight);
  implicitly_convertible<std::shared_ptr<TrafficLight>, RegulatoryElementPtr>();

  enum_<ManeuverType>("ManeuverType")
      .value("Yield", ManeuverType::Yield)
      .value("RightOfWay", ManeuverType::RightOfWay)
      .value("Unknown", ManeuverType::Unknown)
      .export_values();

  class_<RightOfWay, boost::noncopyable, std::shared_ptr<RightOfWay>, bases<RegulatoryElement>>(
      "RightOfWay", "A right of way regulatory element", no_init)
      .def("__init__", make_constructor(&RightOfWay::make, default_call_policies(),
                                        (arg("id"), arg("attributes"), arg("rightOfWayLanelets"), arg("yieldLanelets"),
                                         arg("stopLine") = Optional<LineString3d>{})))
      .add_property(
          "stopLine", +[](RightOfWay& self) { return self.stopLine(); }, &RightOfWay::setStopLine)
      .def("removeStopLine", &RightOfWay::removeStopLine)
      .def("getManeuver", &RightOfWay::getManeuver, "get maneuver for a lanelet")
      .def(
          "rightOfWayLanelets", +[](RightOfWay& self) { return self.rightOfWayLanelets(); })
      .def("addRightOfWayLanelet", &RightOfWay::addRightOfWayLanelet)
      .def("removeRightOfWayLanelet", &RightOfWay::removeRightOfWayLanelet)
      .def(
          "yieldLanelets", +[](RightOfWay& self) { return self.yieldLanelets(); })
      .def("addYieldLanelet", &RightOfWay::addYieldLanelet)
      .def("removeYieldLanelet", &RightOfWay::removeYieldLanelet);
  implicitly_convertible<std::shared_ptr<RightOfWay>, RegulatoryElementPtr>();

  class_<AllWayStop, boost::noncopyable, std::shared_ptr<AllWayStop>, bases<RegulatoryElement>>(
      "AllWayStop", "An all way stop regulatory element", no_init)
      .def("__init__", make_constructor(&AllWayStop::make, default_call_policies(),
                                        (arg("id"), arg("attributes"), arg("lltsWithStop"),
                                         arg("signs") = Optional<LineStringsOrPolygons3d>{})))
      .def(
          "lanelets", +[](AllWayStop& self) { return self.lanelets(); })
      .def(
          "stopLines", +[](AllWayStop& self) { return self.stopLines(); })
      .def(
          "trafficSigns", +[](AllWayStop& self) { return self.trafficSigns(); })
      .def("addTrafficSign", &AllWayStop::addTrafficSign)
      .def("removeTrafficSign", &AllWayStop::removeTrafficSign)
      .def("addLanelet", &AllWayStop::addLanelet)
      .def("removeLanelet", &AllWayStop::removeLanelet);
  implicitly_convertible<std::shared_ptr<AllWayStop>, RegulatoryElementPtr>();

  class_<TrafficSignsWithType, std::shared_ptr<TrafficSignsWithType>>("TrafficSignsWithType", no_init)
      .def("__init__", make_constructor(+[](LineStringsOrPolygons3d ls) {
             return std::make_shared<TrafficSignsWithType>(TrafficSignsWithType{std::move(ls)});
           }))
      .def("__init__", make_constructor(+[](LineStringsOrPolygons3d ls, std::string type) {
             return std::make_shared<TrafficSignsWithType>(TrafficSignsWithType{std::move(ls), std::move(type)});
           }))
      .def_readwrite("trafficSigns", &TrafficSignsWithType::trafficSigns)
      .def_readwrite("type", &TrafficSignsWithType::type);

  class_<TrafficSign, boost::noncopyable, std::shared_ptr<TrafficSign>, bases<RegulatoryElement>>(
      "TrafficSign", "A traffic sign regulatory element", no_init)
      .def("__init__", make_constructor(&TrafficSign::make, default_call_policies(),
                                        (arg("id"), arg("attributes"), arg("trafficSigns"),
                                         arg("cancellingTrafficSigns") = TrafficSignsWithType{},
                                         arg("refLines") = LineStrings3d(), arg("cancelLines") = LineStrings3d())))
      .def(
          "trafficSigns", +[](TrafficSign& self) { return self.trafficSigns(); })
      .def(
          "cancellingTrafficSigns", +[](TrafficSign& self) { return self.cancellingTrafficSigns(); })
      .def(
          "refLines", +[](TrafficSign& self) { return self.refLines(); })
      .def(
          "cancelLines", +[](TrafficSign& self) { return self.cancelLines(); })
      .def("addTrafficSign", &TrafficSign::addTrafficSign)
      .def("removeTrafficSign", &TrafficSign::removeTrafficSign)
      .def("addRefLine", &TrafficSign::addRefLine)
      .def("removeRefLine", &TrafficSign::removeRefLine)
      .def("addCancellingTrafficSign", &TrafficSign::addCancellingTrafficSign)
      .def("removeCancellingTrafficSign", &TrafficSign::removeCancellingTrafficSign)
      .def("addCancellingRefLine", &TrafficSign::addCancellingRefLine)
      .def("removeCancellingRefLine", &TrafficSign::removeCancellingRefLine)
      .def("type", &TrafficSign::type)
      .def("cancelTypes", &TrafficSign::cancelTypes);

  implicitly_convertible<std::shared_ptr<TrafficSign>, RegulatoryElementPtr>();

  implicitly_convertible<std::shared_ptr<SpeedLimit>, RegulatoryElementPtr>();

  class_<SpeedLimit, boost::noncopyable, std::shared_ptr<SpeedLimit>, bases<TrafficSign>>(  // NOLINT
      "SpeedLimit", "A speed limit regulatory element", no_init)
      .def("__init__", make_constructor(&TrafficSign::make, default_call_policies(),
                                        (arg("id"), arg("attributes"), arg("trafficSigns"),
                                         arg("cancellingTrafficSigns") = TrafficSignsWithType{},
                                         arg("refLines") = LineStrings3d(), arg("cancelLines") = LineStrings3d())));

  class_<PrimitiveLayer<Area>, boost::noncopyable>("PrimitiveLayerArea", no_init);        // NOLINT
  class_<PrimitiveLayer<Lanelet>, boost::noncopyable>("PrimitiveLayerLanelet", no_init);  // NOLINT

  wrapLayer<AreaLayer, bases<PrimitiveLayer<Area>>>("AreaLayer")
      .def(
          "findUsages", +[](AreaLayer& self, RegulatoryElementPtr& e) { return self.findUsages(e); })
      .def(
          "findUsages", +[](AreaLayer& self, LineString3d& ls) { return self.findUsages(ls); });
  wrapLayer<LaneletLayer, bases<PrimitiveLayer<Lanelet>>>("LaneletLayer")
      .def(
          "findUsages", +[](LaneletLayer& self, RegulatoryElementPtr& e) { return self.findUsages(e); })
      .def(
          "findUsages", +[](LaneletLayer& self, LineString3d& ls) { return self.findUsages(ls); });
  wrapLayer<PolygonLayer>("PolygonLayer")
      .def(
          "findUsages", +[](PolygonLayer& self, Point3d& p) { return self.findUsages(p); });
  wrapLayer<LineStringLayer>("LineStringLayer")
      .def(
          "findUsages", +[](LineStringLayer& self, Point3d& p) { return self.findUsages(p); });
  wrapLayer<PointLayer>("PointLayer");
  wrapLayer<RegulatoryElementLayer>("RegulatoryElementLayer");

  class_<LaneletMapLayers, boost::noncopyable>("LaneletMapLayers", "Container for the layers of a lanelet map")
      .def_readonly("laneletLayer", &LaneletMap::laneletLayer, "Lanelets")
      .def_readonly("areaLayer", &LaneletMap::areaLayer)
      .def_readonly("regulatoryElementLayer", &LaneletMap::regulatoryElementLayer)
      .def_readonly("lineStringLayer", &LaneletMap::lineStringLayer)
      .def_readonly("polygonLayer", &LaneletMap::polygonLayer)
      .def_readonly("pointLayer", &LaneletMap::pointLayer);

  class_<LaneletMap, bases<LaneletMapLayers>, LaneletMapPtr, boost::noncopyable>(
      "LaneletMap", "Object for managing a lanelet map", init<>("LaneletMap()"))
      .def("add", selectAdd<Point3d>())
      .def("add", selectAdd<Lanelet>())
      .def("add", selectAdd<Area>())
      .def("add", selectAdd<LineString3d>())
      .def("add", selectAdd<Polygon3d>())
      .def("add", selectAdd<const RegulatoryElementPtr&>());
  register_ptr_to_python<LaneletMapConstPtr>();

  class_<LaneletSubmap, bases<LaneletMapLayers>, LaneletSubmapPtr, boost::noncopyable>(
      "LaneletSubmap", "Object for managing parts of a lanelet map", init<>("LaneletSubmap()"))
      .def(
          "laneletMap", +[](LaneletSubmap& self) { return LaneletMapPtr{self.laneletMap()}; })
      .def("add", selectSubmapAdd<Point3d>())
      .def("add", selectSubmapAdd<Lanelet>())
      .def("add", selectSubmapAdd<Area>())
      .def("add", selectSubmapAdd<LineString3d>())
      .def("add", selectSubmapAdd<Polygon3d>())
      .def("add", selectSubmapAdd<const RegulatoryElementPtr&>());
  register_ptr_to_python<LaneletSubmapConstPtr>();

  def("getId", static_cast<Id (&)()>(utils::getId), "Returns a unique id");
  def("registerId", &utils::registerId, "Registers an id");

  def("createMapFromPoints", createMapWrapper<Points3d>, "Create map from primitives");
  def("createMapFromLineStrings", createMapWrapper<LineStrings3d>, "Create map from primitives");
  def("createMapFromPolygons", createMapWrapper<Polygons3d>, "Create map from primitives");
  def("createMapFromLanelets", createMapWrapper<Lanelets>, "Create map from primitives");
  def("createMapFromAreas", createMapWrapper<Areas>, "Create map from primitives");

  def("createSubmapFromPoints", createSubmapWrapper<Points3d>, "Create sbumap from primitives");
  def("createSubmapFromLineStrings", createSubmapWrapper<LineStrings3d>, "Create submap from primitives");
  def("createSubmapFromPolygons", createSubmapWrapper<Polygons3d>, "Create submap from primitives");
  def("createSubmapFromLanelets", createSubmapWrapper<Lanelets>, "Create submap from primitives");
  def("createSubmapFromAreas", createSubmapWrapper<Areas>, "Create submap from primitives");
}
