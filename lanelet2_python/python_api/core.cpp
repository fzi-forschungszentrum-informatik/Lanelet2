#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <boost/python/object_core.hpp>
#include <boost/python/return_by_value.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <sstream>
#include <stdexcept>

#include "lanelet2_core/Attribute.h"
#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/Area.h"
#include "lanelet2_python/internal/converter.h"

using namespace boost::python;
using namespace lanelet;

namespace {
void formatHelper(std::ostream& os) {}

template <typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void formatHelper(std::ostream& os, const std::string& s, const Args&... Args_) {
  if (!s.empty()) {
    os << ", ";
  }
  os << s;
  formatHelper(os, Args_...);
}

template <typename T, typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void formatHelper(std::ostream& os, const T& next, const Args&... Args_) {
  os << ", ";
  os << next;
  formatHelper(os, Args_...);
}

template <typename T, typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void format(std::ostream& os, const T& first, const Args&... Args_) {
  os << first;
  formatHelper(os, Args_...);
}

template <typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
std::string makeRepr(const char* name, const Args&... Args_) {
  std::ostringstream os;
  os << name << '(';
  format(os, Args_...);
  os << ')';
  return os.str();
}

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
    auto* pyStr = PyUnicode_AsUTF8String(objPtr);
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
    c.add_property("id", &PrimT::id, &PrimT::setId,
                   "Unique ID of this primitive. 0 is a special value for temporary objects.");
    c.add_property("attributes", getRefFunc(attr), setAttributeWrapper<PrimT>,
                   "The attributes of this primitive as key value types. Behaves like a dictionary.");
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
    c.add_property("id", &PrimT::id, "Unique ID of this primitive. 0 is a special value for temporary objects.");
    const AttributeMap& (PrimT::*attr)() const = &PrimT::attributes;
    c.add_property("attributes", getRefFunc(attr),
                   "The attributes of this primitive as key value types. Behaves like a dictionary.");
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
    c.def("__iter__", iterator<LsT>())
        .def("__len__", &LsT::size, "Number of points in this linestring")
        .def("__iter__", iterator<LsT>())
        .def("inverted", &LsT::inverted, "Returns whether this is an inverted linestring");
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
        .def("append", &LsT::push_back, "Appends a new point at the end of this linestring", arg("point"))
        .def("__iter__", iterator<LsT>())
        .def("__len__", &LsT::size, "Number of points in this linestring")
        .def("inverted", &LsT::inverted, "Returns whether this is an inverted linestring");
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
        .def("items", MapItem<T>::items, "Iterates over the key-value pairs")
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
  return class_<LayerT, boost::noncopyable, ClassArgs...>(layerName,
                                                          "Primitive layer in a LaneletMap and LaneletSubmap", no_init)
      .def("exists", &LayerT::exists, arg("id"), "Check if a primitive ID exists")
      .def("__contains__", &LayerT::exists, arg("id"), "Check if a primitive ID exists")
      .def(
          "__contains__",
          +[](LayerT& self, const typename LayerT::PrimitiveT& elem) { return self.exists(utils::getId(elem)); },
          arg("elem"), "Check if a primitive with this ID exists")
      .def("get", get, arg("id"), "Get a primitive with this ID")
      .def("__iter__", iterator<LayerT>(), "Iterate primitives in this layer in arbitrary order")
      .def("__len__", &LayerT::size, "Number of items in this layer")
      .def(
          "__getitem__", +[](LayerT& self, Id idx) { return self.get(idx); }, arg("id"),
          "Retrieve an element by its ID")
      .def("search", search, arg("boundingBox"), "Search in a search area defined by a 2D bounding box")
      .def("nearest", nearest, (arg("point"), arg("n") = 1), "Gets a list of the nearest n primitives to a given point")
      .def("uniqueId", &LayerT::uniqueId, "Retrieve an ID that not yet used in this layer");
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

template <typename T>
object optionalToObject(const Optional<T>& v) {
  return v ? object(*v) : object();
}

template <typename T>
Optional<T> objectToOptional(const object& o) {
  return o == object() ? Optional<T>{} : Optional<T>{extract<T>(o)()};
}

std::string repr(const object& o) {
  object repr = import("builtins").attr("repr");
  return call<std::string>(repr.ptr(), o);
}

std::string repr(const AttributeMap& a) {
  if (a.empty()) {
    return {};
  }
  return repr(object(a));
}

std::string repr(const RegulatoryElementConstPtrs& regelems) {
  if (regelems.empty()) {
    return {};
  }
  return repr(list(regelems));
}
}  // namespace

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  class_<BasicPoint2d>("BasicPoint2d", "A simple 2D point", init<double, double>((arg("x") = 0., arg("y") = 0.)))
      .def(init<>("BasicPoint2d()"))
      .add_property("x", getXWrapper<BasicPoint2d>, setXWrapper<BasicPoint2d>, "x coordinate")
      .add_property("y", getYWrapper<BasicPoint2d>, setYWrapper<BasicPoint2d>, "y coordinate")
      .def("__add__", addWrapper<BasicPoint2d>)
      .def("__sub__", subWrapper<BasicPoint2d>)
      .def("__mul__", mulWrapper<BasicPoint2d, double>)
      .def("__rmul__", mulWrapper<BasicPoint2d, double>)
      .def("__div__", divWrapper<BasicPoint2d, double>)
      .def(
          "__repr__", +[](BasicPoint2d p) { return makeRepr("BasicPoint2d", p.x(), p.y()); })
      .def(self_ns::str(self_ns::self));

  class_<Eigen::Vector2d>("Vector2d", "A simple point", no_init)
      .add_property("x", getXWrapper<BasicPoint2d>, setXWrapper<BasicPoint2d>, "x coordinate")
      .add_property("y", getYWrapper<BasicPoint2d>, setYWrapper<BasicPoint2d>, "y coordinate")
      .def("__add__", addWrapper<BasicPoint2d>)
      .def("__sub__", subWrapper<BasicPoint2d>)
      .def("__mul__", mulWrapper<BasicPoint2d, double>)
      .def("__rmul__", mulWrapper<BasicPoint2d, double>)
      .def("__div__", divWrapper<BasicPoint2d, double>)
      .def(
          "__repr__", +[](Eigen::Vector2d p) { return makeRepr("Vector2d", p.x(), p.y()); })
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
      .def(
          "__repr__", +[](BasicPoint3d p) { return makeRepr("BasicPoint3d", p.x(), p.y(), p.z()); })
      .def(self_ns::str(self_ns::self));

  class_<BoundingBox2d>("BoundingBox2d",
                        init<BasicPoint2d, BasicPoint2d>(
                            (arg("min"), arg("max")),
                            "Initialize box with its minimum point (lower left) and its maximum (upper right) corner"))
      .add_property(
          "min",
          make_function(
              +[](BoundingBox2d& self) -> BasicPoint2d& { return self.min(); }, return_internal_reference<>()),
          +[](BoundingBox2d& self, const BasicPoint2d& p) { self.min() = p; }, "Minimum corner (lower left)")
      .add_property(
          "max",
          make_function(
              +[](BoundingBox2d& self) -> BasicPoint2d& { return self.max(); }, return_internal_reference<>()),
          +[](BoundingBox2d& self, const BasicPoint2d& p) { self.max() = p; }, "Maximum corner (upper right)")
      .def(
          "__repr__", +[](BoundingBox2d box) {
            return makeRepr("BoundingBox2d", repr(object(box.min())), repr(object(box.max())));
          });

  class_<BoundingBox3d>("BoundingBox3d",
                        init<BasicPoint3d, BasicPoint3d>(
                            (arg("min"), arg("max")),
                            "Initialize box with its minimum (lower laft) and its maximum (upper right) corner"))
      .add_property(
          "min",
          make_function(
              +[](BoundingBox3d& self) -> BasicPoint3d& { return self.min(); }, return_internal_reference<>{}),
          +[](BoundingBox3d& self, const BasicPoint3d& p) { self.min() = p; }, "Minimum corner")
      .add_property(
          "max",
          make_function(
              +[](BoundingBox3d& self) -> BasicPoint3d& { return self.max(); }, return_internal_reference<>{}),
          +[](BoundingBox3d& self, const BasicPoint3d& p) { self.max() = p; }, "Maximum corner")
      .def(
          "__repr__", +[](const BoundingBox3d& box) {
            return makeRepr("BoundingBox3d", repr(object(box.min())), repr(object(box.max())));
          });

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
  VectorToListConverter<Ids>();
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
      .fromPython<InnerBounds>()
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

  class_<AttributeMap>("AttributeMap", "Stores attributes as key-value pairs. Behaves similar to a dict.",
                       init<>("Create an empty attriute map"))
      .def(IsHybridMap<AttributeMap>())
      .def(self_ns::str(self_ns::self))
      .def(
          "__repr__", +[](const AttributeMap& a) { return makeRepr("AttributeMap", repr(dict(a))); });

  class_<RuleParameterMap>("RuleParameterMap",
                           "Used by RegulatoryElement. Works like a dictionary that maps roles (strings) to a list of "
                           "primitives with that role (parameters).",
                           init<>("Create empty rule parameter map"))
      .def(IsHybridMap<RuleParameterMap>())
      .def(
          "__repr__", +[](const RuleParameterMap& r) { return makeRepr("RuleParameterMap", repr(dict(r))); });

  class_<ConstRuleParameterMap>("ConstRuleParameterMap", init<>("ConstRuleParameterMap()"))
      .def(IsHybridMap<ConstRuleParameterMap>())
      .def(
          "__repr__", +[](const RuleParameterMap& r) { return makeRepr("RuleParameterMap", repr(dict(r))); });

  class_<ConstPoint2d>("ConstPoint2d",
                       "Immutable 2D point primitive. It can not be instanciated from python but is returned from a "
                       "few lanelet2 algorithms",
                       no_init)
      .def(IsConstPrimitive<ConstPoint2d>())
      .add_property("x", getXWrapper<ConstPoint2d>, "x coordinate")
      .add_property("y", getYWrapper<ConstPoint2d>, "y coordinate")
      .def("basicPoint", &ConstPoint2d::basicPoint, return_internal_reference<>(),
           "Returns a plain 2D point primitive (no ID, no attributes) for efficient geometry operations");
  class_<Point2d, bases<ConstPoint2d>>(
      "Point2d",
      "Lanelet's 2D point primitive. Directly convertible to a 3D point, because it is just a 2D view on the "
      "existing 3D data. Use lanelet2.geometry.to3D for this.",
      init<Id, BasicPoint3d, AttributeMap>((arg("id"), arg("point"), arg("attributes") = AttributeMap())))
      .def(init<>("Point2d()"))
      .def(init<Point3d>("Point3d()"))
      .def(init<Id, double, double, double, AttributeMap>(
          (arg("id"), arg("x"), arg("y"), arg("z") = 0., arg("attributes") = AttributeMap())))
      .add_property("x", getXWrapper<Point2d>, setXWrapper<Point2d>, "x coordinate")
      .add_property("y", getYWrapper<Point2d>, setYWrapper<Point2d>, "y coordinate")
      .def(
          "__repr__", +[](const Point2d& p) { return makeRepr("Point2d", p.id(), p.x(), p.y(), repr(p.attributes())); })
      .def(IsPrimitive<Point2d>());

  class_<ConstPoint3d>("ConstPoint3d",
                       "Immutable 3D point primitive. It can not be instanciated from python but is returned from a "
                       "few lanelet2 algorithms",
                       no_init)
      .def(IsConstPrimitive<ConstPoint3d>())
      .add_property("x", getXWrapper<ConstPoint3d>, "x coordinate")
      .add_property("y", getYWrapper<ConstPoint3d>, "y coordinate")
      .add_property("z", getZWrapper<ConstPoint3d>, "z coordinate")
      .def(
          "__repr__",
          +[](const ConstPoint3d& p) {
            return makeRepr("ConstPoint3d", p.id(), p.x(), p.y(), p.z(), repr(p.attributes()));
          })
      .def("basicPoint", &ConstPoint3d::basicPoint, return_internal_reference<>(),
           "Returns a plain 3D point primitive (no ID, no attributes) for efficient geometry operations");

  class_<Point3d, bases<ConstPoint3d>>(
      "Point3d",
      "Lanelets 3D point primitive. Directly convertible to a 2D point, which shares the same view on the data. Use "
      "lanelet2.geometry.to2D for this.",
      init<Id, BasicPoint3d, AttributeMap>((arg("id"), arg("point"), arg("attributes") = AttributeMap())))
      .def(init<>("Create a 3D point with ID 0 (invalid ID) at the origin"))
      .def(init<Point2d>("Create a 3D point from a 2D point"))
      .def(init<Id, double, double, double, AttributeMap>(
          (arg("id"), arg("x"), arg("y"), arg("z") = 0., arg("attributes") = AttributeMap())))
      .add_property("x", getXWrapper<Point3d>, setXWrapper<Point3d>, "x coordinate")
      .add_property("y", getYWrapper<Point3d>, setYWrapper<Point3d>, "y coordinate")
      .add_property("z", getZWrapper<Point3d>, setZWrapper<Point3d>, "z coordinate")
      .def(
          "__repr__",
          +[](const Point3d& p) { return makeRepr("Point3d", p.id(), p.x(), p.y(), p.z(), repr(p.attributes())); })
      .def(IsPrimitive<Point3d>());

  class_<GPSPoint, std::shared_ptr<GPSPoint>>("GPSPoint", "A raw GPS point", no_init)
      .def("__init__", make_constructor(
                           // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
                           +[](double lat, double lon, double alt) {
                             return std::make_shared<GPSPoint>(GPSPoint({lat, lon, alt}));
                           },
                           default_call_policies(), (arg("lat") = 0., arg("lon") = 0., arg("ele") = 0)))
      .def_readwrite("lat", &GPSPoint::lat, "Latitude according to WGS84")
      .def_readwrite("lon", &GPSPoint::lon, "Longitude according to WGS84")
      .def_readwrite("alt", &GPSPoint::ele, "DEPRECATED: Elevation according to WGS84 [m]. Use ele instead.")
      .def_readwrite("ele", &GPSPoint::ele, "Elevation according to WGS84 [m]")
      .def(
          "__repr__", +[](const GPSPoint& p) { return makeRepr("GPSPoint", p.lat, p.lon, p.ele); });

  class_<ConstLineString2d>(
      "ConstLineString2d",
      "Immutable 2d lineString primitive. Behaves similar to a list of points. They cannot be created directly, but "
      "are returned by some lanelet2 functions. Create mutable Linestring3d instead.",
      init<ConstLineString3d>("Create (immutable) 2D Linestring from 3D linestring", (args("linestring"))))
      .def("invert", &ConstLineString2d::invert,
           "Creates a new, inverted linestring from this. This is essentially a view on the existing data, with points "
           "returned in reversed order")
      .def(IsConstLineString<ConstLineString2d>())
      .def(IsConstPrimitive<ConstLineString2d>())
      .def(
          "__repr__", +[](const ConstLineString2d& ls) {
            return makeRepr("ConstLineString2d", ls.id(), repr(list(ls)), repr(ls.attributes()));
          });

  class_<LineString2d, bases<ConstLineString2d>>(
      "LineString2d",
      "Lanelet2s 2D linestring primitive. Has an ID, attributes and points with linear iterpolation in between the "
      "points. Easily convertible to a 3D linestring, "
      "because it is essentially a view on "
      "the same 3D data. Use lanelet2.geometry.to3D for this.",
      init<Id, Points3d, AttributeMap>("Create a linestring from an ID, a list of 3D points and attributes",
                                       (arg("id"), arg("points"), arg("attributes"))))
      .def(init<Id, Points3d>("Create a linestring from an ID and a list of 3D points", (arg("id"), arg("points"))))
      .def(init<LineString3d>("Returns a 3D linestring for the current data. Both share the same data, modifications "
                              "affect both linestrings."))
      .def("invert", &LineString2d::invert,
           "Creates a new, inverted linestring from this. This is essentially a view on the existing data, with "
           "points "
           "returned in reversed order")
      .def(
          "__repr__",
          +[](const ConstLineString3d& ls) {
            return makeRepr("LineString3d", ls.id(), repr(list(ls)), repr(ls.attributes()));
          })
      .def(IsLineString<LineString2d>());

  class_<ConstLineString3d>("ConstLineString3d",
                            "Immutable 3d lineString primitive. They cannot be created directly, but "
                            "are returned by some lanelet2 functions. Create mutable Linestring3d instead. Use "
                            "lanelet2.geometry.to2D to convert to 2D pendant.",
                            init<ConstLineString2d>("Convert 2d linestring to 3D linestring"))
      .def(init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def("invert", &ConstLineString3d::invert,
           "Creates a new, inverted linestring from this. This is essentially a view on the existing data, with "
           "points "
           "returned in reversed order")
      .def(
          "__repr__",
          +[](const ConstLineString3d& ls) {
            return makeRepr("ConstLineString3d", ls.id(), repr(list(ls)), repr(ls.attributes()));
          })
      .def(IsConstLineString<ConstLineString3d>())
      .def(IsConstPrimitive<ConstLineString3d>());

  class_<LineString3d, bases<ConstLineString3d>>(
      "LineString3d",
      "Lanelet's 3d lineString primitive. Has an ID, attribtues and points. Accessing individual points works similar "
      "to python lists. Create mutable Linestring3d instead. Use lanelet2.geometry.to2D to convert to Linestring2d."
      "convert to Linestring2d.",
      init<Id, Points3d, AttributeMap>((arg("id") = 0, arg("points") = Points3d{}, arg("attributes") = AttributeMap())))
      .def(init<LineString2d>(arg("linestring"),
                              "Converts a 2D linestring to a 3D linestring, sharing the same underlying data."))
      .def("invert", &LineString3d::invert,
           "Creates a new, inverted linestring from this. This is essentially a view on the existing data, with "
           "points returned in reversed order")
      .def(
          "__repr__",
          +[](const LineString3d& ls) {
            return makeRepr("LineString3d", ls.id(), repr(list(ls)), repr(ls.attributes()));
          })
      .def(IsLineString<LineString3d>())
      .def(IsPrimitive<LineString3d>());

  class_<ConstHybridLineString2d>(
      "ConstHybridLineString2d",
      "A 2D Linestring that behaves like a normal BasicLineString (i.e. returns BasicPoints), "
      "but still has an ID and attributes.",
      init<ConstLineString2d>(arg("linestring"), "Create from a 2D linestring"))
      .def("invert", &ConstHybridLineString2d::invert,
           "Creates a new, inverted linestring from this. This is essentially a view on the existing data, with "
           "points returned in reversed order")
      .def(
          "__repr__",
          +[](const ConstHybridLineString2d& ls) {
            return makeRepr("ConstHybridLineString2d", ls.id(), repr(list(ls)), repr(ls.attributes()));
          })
      .def(IsConstLineString<ConstHybridLineString2d, false>())
      .def(IsConstPrimitive<ConstHybridLineString2d>());

  class_<ConstHybridLineString3d>(
      "ConstHybridLineString3d",
      "A 3D Linestring that behaves like a normal BasicLineString (i.e. returns BasicPoints), "
      "but still has an ID and attributes.",
      init<ConstLineString3d>(arg("linestring"), "Create from a 3D linestring"))
      .def("invert", &ConstHybridLineString3d::invert,
           "Creates a new, inverted linestring from this. This is essentially a view on the existing data, with "
           "points returned in reversed order")
      .def(
          "__repr__",
          +[](const ConstHybridLineString3d& ls) {
            return makeRepr("ConstHybridLineString3d", ls.id(), repr(list(ls)), repr(ls.attributes()));
          })
      .def(IsConstLineString<ConstHybridLineString3d, false>())
      .def(IsConstPrimitive<ConstHybridLineString3d>());

  class_<CompoundLineString2d>("CompoundLineString2d",
                               "A LineString composed of multiple linestrings (i.e. it provides access to the points "
                               "in these linestrings in order)",
                               init<ConstLineStrings2d>(arg("linestrings"), "Create from a list of LineString2d"))
      .def(IsConstLineString<CompoundLineString2d>())
      .def("lineStrings", &CompoundLineString2d::lineStrings, "The linestrings that this linestring consists of")
      .def("ids", &CompoundLineString2d::ids, "List of the IDs of the linestrings");

  class_<CompoundLineString3d>("CompoundLineString3d", "A LineString composed of multiple linestrings",
                               init<ConstLineStrings3d>(arg("linestrings"), "Create from a list of LineString3d"))
      .def(IsConstLineString<CompoundLineString3d>())
      .def("lineStrings", &CompoundLineString3d::lineStrings, "The linestrings that this linestring consists of")
      .def("ids", &CompoundLineString3d::ids, "List of the IDs of the linestrings");

  class_<ConstPolygon2d>(
      "ConstPolygon2d", "A two-dimensional lanelet polygon",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap()),
                                       "Create from an ID, a list of points and optionally attributes"))
      .def(IsConstLineString<ConstPolygon2d>())
      .def(
          "__repr__",
          +[](const ConstPolygon2d& p) {
            return makeRepr("ConstPolygon2d", p.id(), repr(list(p)), repr(p.attributes()));
          })
      .def(IsConstPrimitive<ConstPolygon2d>());

  class_<ConstPolygon3d>(
      "ConstPolygon3d", "A three-dimensional lanelet polygon",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def(
          "__repr__",
          +[](const ConstPolygon3d& p) {
            return makeRepr("ConstPolygon3d", p.id(), repr(list(p)), repr(p.attributes()));
          })
      .def(IsConstLineString<ConstPolygon3d>())
      .def(IsConstPrimitive<ConstPolygon3d>());

  class_<Polygon2d, bases<ConstPolygon2d>>(
      "Polygon2d",
      "A two-dimensional lanelet polygon. Points in clockwise order and open (i.e. start point != end point).",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap()),
                                       "Create from an ID, the boundary points and (optionally) attributes"))
      .def(
          "__repr__",
          +[](const Polygon2d& p) { return makeRepr("Polygon2d", p.id(), repr(list(p)), repr(p.attributes())); })
      .def(IsLineString<Polygon2d>())
      .def(IsPrimitive<Polygon2d>());

  class_<Polygon3d, bases<ConstPolygon3d>>(
      "Polygon3d",
      "A two-dimensional lanelet polygon. Points in clockwise order and open (i.e. start point != end point).",
      init<Id, Points3d, AttributeMap>((arg("id"), arg("points"), arg("attributes") = AttributeMap())))
      .def(
          "__repr__",
          +[](const Polygon3d& p) { return makeRepr("Polygon3d", p.id(), repr(list(p)), repr(p.attributes())); })
      .def(IsLineString<Polygon3d>())
      .def(IsPrimitive<Polygon3d>());

  class_<CompoundPolygon2d>(
      "CompoundPolygon2d", "A 2D polygon composed of multiple linestrings",
      init<ConstLineStrings2d>(arg("linestrings"),
                               "Create a compound polygon from a list of linestrings (in clockwise order)"))
      .def(IsConstLineString<CompoundPolygon2d>())
      .def("lineStrings", &CompoundPolygon2d::lineStrings, "The linestrings in this polygon")
      .def("ids", &CompoundPolygon2d::ids, "List of the ids of the linestrings");

  class_<CompoundPolygon3d>("CompoundPolygon3d", "A 3D polygon composed of multiple linestrings",
                            init<ConstLineStrings3d>())
      .def(IsConstLineString<CompoundPolygon3d>())
      .def("lineStrings", &CompoundPolygon3d::lineStrings, "The linestrings in this polygon")
      .def("ids", &CompoundPolygon3d::ids, "List of the ids of the linestrings");

  class_<ConstLanelet>(
      "ConstLanelet",
      "An immutable lanelet primitive. Consist of a left and right boundary, attributes and a set of "
      "traffic rules that apply here. It is not very useful within python, but returned by some lanelet2 algorihms",
      init<Id, LineString3d, LineString3d, AttributeMap, RegulatoryElementPtrs>(
          (arg("id"), arg("leftBound"), arg("rightBound"), arg("attributes") = AttributeMap(),
           arg("regelems") = RegulatoryElementPtrs{})))
      .def(init<Lanelet>(arg("lanelet"), "Construct from a mutable lanelet"))
      .def(IsConstPrimitive<ConstLanelet>())
      .add_property(
          "centerline", &ConstLanelet::centerline,
          "Centerline of the lanelet (immutable). This is usualy calculated on-the-fly from the left and right "
          "bound. The centerline and all its points have ID 0. The centerline can also be overridden by a custom "
          "centerline, which will usually have nonzero IDs.")
      .add_property("leftBound", &ConstLanelet::leftBound, "Left boundary of the lanelet")
      .add_property("rightBound", &ConstLanelet::rightBound, "Right boundary of the lanelet")
      .add_property("regulatoryElements",
                    make_function(&ConstLanelet::regulatoryElements, return_value_policy<return_by_value>()),
                    "Regulatory elements of the lanelet")
      .def("trafficLights", constRegelemAs<TrafficLight>,
           "Returns the list of traffic light regulatory elements of this lanelet")
      .def("trafficSigns", constRegelemAs<TrafficSign>,
           "Returns a list of traffic sign regulatory elements of this lanelet")
      .def("speedLimits", constRegelemAs<SpeedLimit>,
           "Returns a list of speed limit regulatory elements of this lanelet")
      .def("rightOfWay", constRegelemAs<RightOfWay>, "Returns right of way regulatory elements of this lanelet")
      .def("allWayStop", constRegelemAs<AllWayStop>, "Returns all way stop regulatory elements of this lanelet")
      .def("invert", &ConstLanelet::invert, "Returns inverted lanelet (flipped left/right bound, etc")
      .def("inverted", &ConstLanelet::inverted, "Returns whether this lanelet has been inverted")
      .def("polygon2d", &ConstLanelet::polygon2d, "Outline of this lanelet as 2d polygon")
      .def("polygon3d", &ConstLanelet::polygon3d, "Outline of this lanelet as 3d polygon")
      .def("resetCache", &ConstLanelet::resetCache,
           "Reset the cache. Forces update of the centerline if points have changed. This does not clear a custom "
           "centerline.")
      .def(
          "__repr__", +[](const ConstLanelet& llt) {
            return makeRepr("ConstLanelet", llt.id(), repr(object(llt.leftBound())), repr(object(llt.rightBound())),
                            repr(llt.attributes()), repr(llt.regulatoryElements()));
          });

  auto left = static_cast<LineString3d (Lanelet::*)()>(&Lanelet::leftBound);
  auto right = static_cast<LineString3d (Lanelet::*)()>(&Lanelet::rightBound);
  auto regelems = static_cast<const RegulatoryElementPtrs& (Lanelet::*)()>(&Lanelet::regulatoryElements);

  class_<Lanelet, bases<ConstLanelet>>(
      "Lanelet", "The famous lanelet primitive",
      init<Id, LineString3d, LineString3d, AttributeMap, RegulatoryElementPtrs>(
          (arg("id"), arg("leftBound"), arg("rightBound"), arg("attributes") = AttributeMap(),
           arg("regelems") = RegulatoryElementPtrs{}),
          "Create Lanelet from an ID, its left and right boundary and (optionally) attributes"))
      .def(IsPrimitive<Lanelet>())
      .add_property(
          "centerline", &Lanelet::centerline, &Lanelet::setCenterline,
          "Centerline of the lanelet (immutable). This is usualy calculated on-the-fly from the left and right "
          "bound. The centerline and all its points have ID 0. Assigning a linestring will replace this by a custom, "
          "persistent centerline.")
      .add_property("leftBound", left, &Lanelet::setLeftBound, "Left boundary of the lanelet")
      .add_property("rightBound", right, &Lanelet::setRightBound, "Right boundary of the lanelet")
      .add_property("regulatoryElements", make_function(regelems, return_value_policy<return_by_value>()),
                    "Regulatory elements of the lanelet")
      .def("trafficLights", regelemAs<TrafficLight>,
           "Returns the list of traffic light regulatory elements of this lanelet")
      .def("trafficSigns", regelemAs<TrafficSign>, "Returns a list of traffic sign regulatory elements of this lanelet")
      .def("speedLimits", regelemAs<SpeedLimit>, "Returns a list of speed limit regulatory elements of this lanelet")
      .def("rightOfWay", regelemAs<RightOfWay>, "Returns right of way regulatory elements of this lanelet")
      .def("allWayStop", regelemAs<AllWayStop>, "Returns all way stop regulatory elements of this lanelet")
      .def("addRegulatoryElement", &Lanelet::addRegulatoryElement, arg("regelem"),
           "Adds a new regulatory element to the lanelet")
      .def("removeRegulatoryElement", &Lanelet::removeRegulatoryElement, arg("regelem"),
           "Removes a regulatory element from the lanelet, returns true on success")
      .def("invert", &Lanelet::invert, "Returns inverted lanelet (flipped left/right bound, etc)")
      .def("inverted", &ConstLanelet::inverted, "Returns whether this lanelet has been inverted")
      .def("polygon2d", &ConstLanelet::polygon2d, "Outline of this lanelet as 2D polygon")
      .def("polygon3d", &ConstLanelet::polygon3d, "Outline of this lanelet as 3D polygon")
      .def(
          "__repr__", +[](Lanelet& llt) {
            return makeRepr("Lanelet", llt.id(), repr(object(llt.leftBound())), repr(object(llt.rightBound())),
                            repr(llt.attributes()), repr(ConstLanelet(llt).regulatoryElements()));
          });

  class_<LaneletSequence>("LaneletSequence", "A combined lane formed from multiple lanelets", init<ConstLanelets>())
      .add_property("centerline", &LaneletSequence::centerline, "Centerline of the combined lanelets")
      .add_property("leftBound", &LaneletSequence::leftBound, "Left boundary of the combined lanelets")
      .add_property("rightBound", &LaneletSequence::rightBound, "Right boundary of the combined lanelets")
      .def("invert", &LaneletSequence::invert, "Returns inverted lanelet sequence (flipped left/right bound, etc)")
      .def("polygon2d", &LaneletSequence::polygon2d, "Outline of this lanelet as 2D polygon")
      .def("polygon3d", &LaneletSequence::polygon3d, "Outline of this lanelet as 3D polygon")
      .def("lanelets", &LaneletSequence::lanelets, "Lanelets that make up this lanelet sequence")
      .def("__iter__", iterator<LaneletSequence>())
      .def("__len__", &LaneletSequence::size, "Number of lanelets in this sequence")
      .def("inverted", &LaneletSequence::inverted, "True if this lanelet sequence is inverted")
      .def(
          "__repr__", +[](const LaneletSequence& s) { return makeRepr("LaneletSequence", repr(object(s.lanelets()))); })
      .def("__getitem__", wrappers::getItem<LaneletSequence>, return_internal_reference<>(),
           "Returns a lanelet in the sequence");

  class_<ConstLaneletWithStopLine>("ConstLaneletWithStopLine", "A lanelet with an (optional) stopline", no_init)
      .def("__init__",
           make_constructor(
               +[](Lanelet lanelet, Optional<ConstLineString3d> stopLine) {
                 return std::make_shared<ConstLaneletWithStopLine>(
                     ConstLaneletWithStopLine{std::move(lanelet), std::move(stopLine)});
               },
               default_call_policies(), (arg("lanelet"), arg("stopLine") = Optional<ConstLineString3d>{})),
           "Construct from a lanelet and a stop line")
      .add_property("lanelet", &ConstLaneletWithStopLine::lanelet, "The lanelet")
      .add_property(
          "stopLine", +[](const ConstLaneletWithStopLine& self) { return optionalToObject(self.stopLine); },
          +[](ConstLaneletWithStopLine& self, const object& value) {
            self.stopLine = objectToOptional<LineString3d>(value);
          },
          "The stop line for the lanelet (can be None)");

  class_<LaneletWithStopLine>("LaneletWithStopLine", "A lanelet with a stopline", no_init)
      .def("__init__", make_constructor(
                           +[](Lanelet lanelet, Optional<LineString3d> stopLine) {
                             return std::make_shared<LaneletWithStopLine>(
                                 LaneletWithStopLine{std::move(lanelet), std::move(stopLine)});
                           },
                           default_call_policies(), (arg("lanelet"), arg("stopLine") = Optional<LineString3d>{})))
      .add_property("lanelet", &LaneletWithStopLine::lanelet, "The lanelet")
      .add_property(
          "stopLine", +[](const LaneletWithStopLine& self) { return optionalToObject(self.stopLine); },
          +[](LaneletWithStopLine& self, const object& value) {
            self.stopLine = objectToOptional<LineString3d>(value);
          },
          "The stop line (LineString3d or None)");

  // areas are a bit complicated because the holes are vectors of vectors. For those, we cannot rely on the automatic
  // vector-to-list conversion, because area.innerBounds[0].append(ls) would then operate on a temporary list and do
  // nothing.
  class_<ConstInnerBounds>("ConstInnerBounds", "An immutable list-like type to hold of inner boundaries of an Area",
                           no_init)
      .def("__iter__", iterator<ConstInnerBounds>())
      .def("__len__", &InnerBounds::size, "Number of holes")
      .def("__getitem__", wrappers::getItem<ConstInnerBounds>, return_value_policy<return_by_value>())
      .def(
          "__repr__", +[](const ConstInnerBounds& b) { return repr(list(b)); });

  class_<InnerBounds>("InnerBounds", "A list-like type to hold of inner boundaries of an Area", no_init)
      .def("__setitem__", wrappers::setItem<InnerBounds, LineStrings3d>)
      .def("__delitem__", wrappers::delItem<InnerBounds>)
      .def(
          "append", +[](InnerBounds& self, LineStrings3d ls) { self.push_back(std::move(ls)); }, "Appends a new hole",
          arg("hole"))
      .def("__iter__", iterator<InnerBounds>())
      .def("__len__", &InnerBounds::size, "Number of holes")
      .def("__getitem__", wrappers::getItem<InnerBounds>, return_value_policy<return_by_value>())
      .def(
          "__repr__", +[](const InnerBounds& b) { return repr(list(b)); });

  class_<ConstArea>(
      "ConstArea",
      "Represents an immutable area, potentially with holes, in the map. It is composed of a list of linestrings "
      "that "
      "form the outer boundary and a list of a list of linestrings that represent holes within it.",
      boost::python::init<Id, LineStrings3d, InnerBounds, AttributeMap, RegulatoryElementPtrs>(
          (arg("id"), arg("outerBound"), arg("innerBounds") = InnerBounds{}, arg("attributes") = AttributeMap{},
           arg("regulatoryElements") = RegulatoryElementPtrs{}),
          "Construct an area from an ID, its inner and outer boundaries and attributes"))
      .def(IsConstPrimitive<ConstArea>())
      .add_property("outerBound", &ConstArea::outerBound,
                    "The outer boundary, a list of clockwise oriented linestrings")
      .add_property("innerBounds", &ConstArea::innerBounds,
                    "The inner boundaries (holes), a list of a list of counter-clockwise oriented linestrings")
      .add_property(
          "regulatoryElements", +[](ConstArea& self) { return self.regulatoryElements(); },
          "The regulatory elements of this area")
      .def("outerBoundPolygon", &ConstArea::outerBoundPolygon, "Returns the outer boundary as a CompoundPolygon3d")
      .def(
          "innerBoundPolygon",
          +[](const ConstArea& /*ar*/) {
            throw std::runtime_error("innerBoundPolygon is deprecated. Use innerBoundPolygons instead!");
          },
          "DEPRECATED. Using it throws an exception. Use innerBoundPolygons instead!")
      .def("innerBoundPolygons", &ConstArea::innerBoundPolygons,
           "Returns the inner boundaries as a list of CompoundPolygon3d")
      .def(
          "__repr__", +[](const ConstArea& ar) {
            return makeRepr("ConstArea", ar.id(), repr(list(ar.outerBound())), repr(object(ar.innerBounds())),
                            repr(ar.attributes()), repr(ar.regulatoryElements()));
          });

  class_<Area, bases<ConstArea>>(
      "Area", "Represents an area, potentially with holes, in the map",
      boost::python::init<Id, LineStrings3d, InnerBounds, AttributeMap, RegulatoryElementPtrs>(
          (arg("id"), arg("outerBound"), arg("innerBounds") = InnerBounds{}, arg("attributes") = AttributeMap{},
           arg("regulatoryElements") = RegulatoryElementPtrs{}),
          "Construct an area from an ID, its inner and outer boundaries and attributes"))
      .def(IsPrimitive<Area>())
      .add_property(
          "outerBound", +[](Area& self) { return self.outerBound(); }, &Area::setOuterBound,
          "The outer boundary, a list of clockwise oriented linestrings")
      .add_property(
          "innerBounds",
          make_function(
              // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
              +[](Area& self) -> InnerBounds& { return const_cast<InnerBounds&>(self.innerBounds()); },
              return_internal_reference<>()),
          +[](Area& self, const InnerBounds& lss) { self.setInnerBounds(lss); },
          "The inner boundaries (holes), a list of a list of counter-clockwise oriented linestrings")
      .add_property(
          "regulatoryElements", +[](Area& self) { return self.regulatoryElements(); },
          "The regulatory elements of this area")
      .def("addRegulatoryElement", &Area::addRegulatoryElement, "Appends a new regulatory element", arg("regelem"))
      .def("removeRegulatoryElement", &Area::removeRegulatoryElement,
           "Removes a regulatory element, retunrs true on success", arg("regelem"))
      .def("outerBoundPolygon", &Area::outerBoundPolygon, "Returns the outer boundary as a CompoundPolygon3d")
      .def("innerBoundPolygon", +[](const Area& /*ar*/) { throw std::runtime_error("innerBoundPolygon is deprecated. Use innerBoundPolygons instead!");}, "DEPRECATED. Using it throws an exception. Use innerBoundPolygons instead!")
      .def("innerBoundPolygons", &Area::innerBoundPolygons,
           "Returns the inner boundaries as a list of CompoundPolygon3d")
      .def(
          "__repr__", +[](Area& ar) {
            return makeRepr("Area", ar.id(), repr(list(ar.outerBound())), repr(object(ar.innerBounds())),
                            repr(ar.attributes()), repr(ConstArea(ar).regulatoryElements()));
          });

  using GetParamSig = ConstRuleParameterMap (RegulatoryElement::*)() const;

  class_<RegulatoryElement, boost::noncopyable, RegulatoryElementPtr>(
      "RegulatoryElement",
      "A Regulatory element defines traffic rules that affect a lanelet. This is a abstract base class that is "
      "implemented e.g. by the TrafficLight class.",
      no_init)
      .def(IsConstPrimitive<RegulatoryElement>())
      .add_property("id", &RegulatoryElement::id, &RegulatoryElement::setId)
      .add_property("parameters", static_cast<GetParamSig>(&RegulatoryElement::getParameters),
                    "The parameters (ie traffic signs, lanelets) that affect "
                    "this RegulatoryElement")
      .add_property("roles", &RegulatoryElement::roles, "A list of roles (strings) used in this regulatory element")
      .def("find", &RegulatoryElement::find<ConstRuleParameter>, arg("id"),
           "Returns a primitive with matching ID, else None")
      .def("__len__", &RegulatoryElement::size)
      .def(
          "__repr__", +[](RegulatoryElement& r) {
            return makeRepr("RegulatoryElement", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
          });
  register_ptr_to_python<RegulatoryElementConstPtr>();

  class_<TrafficLight, boost::noncopyable, std::shared_ptr<TrafficLight>, bases<RegulatoryElement>>(
      "TrafficLight", "A traffic light regulatory element", no_init)
      .def("__init__", make_constructor(&TrafficLight::make, default_call_policies(),
                                        (arg("id"), arg("attributes"), arg("trafficLights"),
                                         arg("stopLine") = Optional<LineString3d>())))
      .add_property(
          "stopLine", +[](TrafficLight& self) { return self.stopLine(); }, &TrafficLight::setStopLine,
          "The stop line of for this TrafficLight (a LineString or None)")
      .add_property("removeStopLine", &TrafficLight::removeStopLine, "Clear the stop line")
      .add_property(
          "trafficLights", +[](TrafficLight& self) { return self.trafficLights(); },
          "Traffic lights. Should all have the same phase.")
      .def("addTrafficLight", &TrafficLight::addTrafficLight,
           "Add a traffic light. Either a linestring from the left edge to the right edge or an area with the outline "
           "of the traffic light.")
      .def("removeTrafficLight", &TrafficLight::removeTrafficLight,
           "Removes a given traffic light, returns true on success")
      .def(
          "__repr__", +[](TrafficLight& r) {
            return makeRepr("TrafficLight", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
          });
  implicitly_convertible<std::shared_ptr<TrafficLight>, RegulatoryElementPtr>();

  enum_<ManeuverType>("ManeuverType")
      .value("Yield", ManeuverType::Yield)
      .value("RightOfWay", ManeuverType::RightOfWay)
      .value("Unknown", ManeuverType::Unknown)
      .export_values();

  class_<RightOfWay, boost::noncopyable, std::shared_ptr<RightOfWay>, bases<RegulatoryElement>>(
      "RightOfWay", "A right of way regulatory element", no_init)
      .def("__init__",
           make_constructor(&RightOfWay::make, default_call_policies(),
                            (arg("id"), arg("attributes"), arg("rightOfWayLanelets"), arg("yieldLanelets"),
                             arg("stopLine") = Optional<LineString3d>{})),
           "Creates a right of way regulatory element with an ID from attributes, the list lanelets with right of "
           "way, "
           "the list of yielding lanelets and an optional stop line")
      .add_property(
          "stopLine", +[](RightOfWay& self) { return self.stopLine(); }, &RightOfWay::setStopLine,
          "The stop line (can be None)")
      .def("removeStopLine", &RightOfWay::removeStopLine, "Clear the stop line")
      .def("getManeuver", &RightOfWay::getManeuver, "Get maneuver for a lanelet")
      .def(
          "rightOfWayLanelets", +[](RightOfWay& self) { return self.rightOfWayLanelets(); },
          "Returns the lanelets that have the right of way")
      .def("addRightOfWayLanelet", &RightOfWay::addRightOfWayLanelet, arg("lanelet"),
           "Add another Lanelet that has the right of way. The Lanelet should also reference this regulatory element.")
      .def("removeRightOfWayLanelet", &RightOfWay::removeRightOfWayLanelet, arg("lanelet"),
           "Removes the right of way for this lanelet, returns true on success.")
      .def(
          "yieldLanelets", +[](RightOfWay& self) { return self.yieldLanelets(); })
      .def("addYieldLanelet", &RightOfWay::addYieldLanelet, arg("lanelet"),
           "Adds another lanelet that has to yield. The lanelet should also reference this regolatory element.")
      .def("removeYieldLanelet", &RightOfWay::removeYieldLanelet, arg("lanelet"),
           "Removes the yielding lanelet, returns true on success.")
      .def(
          "__repr__", +[](RightOfWay& r) {
            return makeRepr("RightOfWay", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
          });
  implicitly_convertible<std::shared_ptr<RightOfWay>, RegulatoryElementPtr>();

  class_<AllWayStop, boost::noncopyable, std::shared_ptr<AllWayStop>, bases<RegulatoryElement>>(
      "AllWayStop", "An all way stop regulatory element", no_init)
      .def("__init__",
           make_constructor(
               &AllWayStop::make, default_call_policies(),
               (arg("id"), arg("attributes"), arg("lltsWithStop"), arg("signs") = LineStringsOrPolygons3d{})),
           "Constructs an all way stop regulatory element with an ID and attributes from a list of lanelets with "
           "their "
           "(optional) stop line and an optional list of traffic signs for this rule")
      .def(
          "lanelets", +[](AllWayStop& self) { return self.lanelets(); }, "Returns the lanelets")
      .def(
          "stopLines", +[](AllWayStop& self) { return self.stopLines(); },
          "Returns the stop lines (same order as the lanelets)")
      .def(
          "trafficSigns", +[](AllWayStop& self) { return self.trafficSigns(); },
          "Returns the list of traffic signs for this rule")
      .def("addTrafficSign", &AllWayStop::addTrafficSign, arg("sign"),
           "Adds another traffic sign (a Linestring3d from their left to the right edge or a Polygon3d with the "
           "outline)")
      .def("removeTrafficSign", &AllWayStop::removeTrafficSign, arg("sign"),
           "Removes a traffic sign, returns True on success")
      .def("addLanelet", &AllWayStop::addLanelet, arg("lanelet"),
           "Adds another lanelet to the all way stop. The lanelet should also reference this all way stop.")
      .def("removeLanelet", &AllWayStop::removeLanelet, arg("lanelet"),
           "Removes a lanelet from the all way stop, returns True on success.")
      .def(
          "__repr__", +[](AllWayStop& r) {
            return makeRepr("AllWayStop", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
          });
  implicitly_convertible<std::shared_ptr<AllWayStop>, RegulatoryElementPtr>();

  class_<TrafficSignsWithType, std::shared_ptr<TrafficSignsWithType>>(
      "TrafficSignsWithType", "Combines a traffic sign with its type, used for the TrafficSign regulatory element",
      no_init)
      .def("__init__", make_constructor(+[](LineStringsOrPolygons3d ls) {
             return std::make_shared<TrafficSignsWithType>(TrafficSignsWithType{std::move(ls)});
           }),
           "Construct from a Linestring/Polygon with a type deduced from the attributes.")
      .def("__init__", make_constructor(+[](LineStringsOrPolygons3d ls, std::string type) {
             return std::make_shared<TrafficSignsWithType>(TrafficSignsWithType{std::move(ls), std::move(type)});
           }),
           "Construct from a Linestring/Polygon with an explicit type")
      .def_readwrite("trafficSigns", &TrafficSignsWithType::trafficSigns)
      .def_readwrite("type", &TrafficSignsWithType::type);

  class_<TrafficSign, boost::noncopyable, std::shared_ptr<TrafficSign>, bases<RegulatoryElement>>(
      "TrafficSign", "A generic traffic sign regulatory element", no_init)
      .def("__init__",
           make_constructor(&TrafficSign::make, default_call_policies(),
                            (arg("id"), arg("attributes"), arg("trafficSigns"),
                             arg("cancellingTrafficSigns") = TrafficSignsWithType{}, arg("refLines") = LineStrings3d(),
                             arg("cancelLines") = LineStrings3d())),
           "Constructs a traffic sign withan ID and attributes from a traffic signs with type object and optionally "
           "cancelling traffic signs, lines from where the rule starts and lines where it ends")
      .def(
          "trafficSigns", +[](TrafficSign& self) { return self.trafficSigns(); },
          "Get a list of all traffic signs (linestrings or polygons)")
      .def(
          "cancellingTrafficSigns", +[](TrafficSign& self) { return self.cancellingTrafficSigns(); },
          "Get a list of all cancelling traffic signs (linestrings or polygons). These are the signs that mark the "
          "end "
          "of a rule. E.g. a sign that cancels a speed limit.")
      .def(
          "refLines", +[](TrafficSign& self) { return self.refLines(); },
          "List of linestrings after which the traffic rule becomes valid")
      .def(
          "cancelLines", +[](TrafficSign& self) { return self.cancelLines(); },
          "List of linestrings after which the rule becomes invalid")
      .def("addTrafficSign", &TrafficSign::addTrafficSign, arg("trafficSign"),
           "Add another traffic sign (linestring or polygon)")
      .def("removeTrafficSign", &TrafficSign::removeTrafficSign, arg("trafficSign"),
           "Remove a traffic sign, returns True on success")
      .def("addRefLine", &TrafficSign::addRefLine, arg("refLine"), "Add a ref line")
      .def("removeRefLine", &TrafficSign::removeRefLine, arg("refLine"), "Remove a ref line, returns True on success")
      .def("addCancellingTrafficSign", &TrafficSign::addCancellingTrafficSign, arg("cancellingTrafficSign"),
           "Add a cancelling traffic sign")
      .def("removeCancellingTrafficSign", &TrafficSign::removeCancellingTrafficSign, arg("cancellingTrafficSign"),
           "Remove cancelling traffic sign, return True on success")
      .def("addCancellingRefLine", &TrafficSign::addCancellingRefLine, arg("cancelLine"), "Add a cancelling ref line")
      .def("removeCancellingRefLine", &TrafficSign::removeCancellingRefLine, arg("cancelLine"),
           "Remove a cancelling ref line, returns True on success")
      .def("type", &TrafficSign::type,
           "Returns the type (string) of the traffic sign that start this rule. This is deduced from the traffic sign "
           "itself or "
           "the explicitly set type. All signs are assumed to have the same type.")
      .def("cancelTypes", &TrafficSign::cancelTypes,
           "Returns types of the cancelling traffic signs (a list of strings) if it exists.")
      .def(
          "__repr__", +[](TrafficSign& r) {
            return makeRepr("TrafficSign", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
          });

  implicitly_convertible<std::shared_ptr<TrafficSign>, RegulatoryElementPtr>();

  implicitly_convertible<std::shared_ptr<SpeedLimit>, RegulatoryElementPtr>();

  class_<SpeedLimit, boost::noncopyable, std::shared_ptr<SpeedLimit>, bases<TrafficSign>>(  // NOLINT
      "SpeedLimit", "A speed limit regulatory element. This is a special case of a traffic sign regulatory element.",
      no_init)
      .def("__init__", make_constructor(&TrafficSign::make, default_call_policies(),
                                        (arg("id"), arg("attributes"), arg("trafficSigns"),
                                         arg("cancellingTrafficSigns") = TrafficSignsWithType{},
                                         arg("refLines") = LineStrings3d(), arg("cancelLines") = LineStrings3d())))
      .def(
          "__repr__", +[](SpeedLimit& r) {
            return makeRepr("SpeedLimit", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
          });

  class_<PrimitiveLayer<Area>, boost::noncopyable>("PrimitiveLayerArea", no_init);        // NOLINT
  class_<PrimitiveLayer<Lanelet>, boost::noncopyable>("PrimitiveLayerLanelet", no_init);  // NOLINT

  wrapLayer<AreaLayer, bases<PrimitiveLayer<Area>>>("AreaLayer")
      .def(
          "findUsages", +[](AreaLayer& self, RegulatoryElementPtr& e) { return self.findUsages(e); }, arg("regelem"),
          "Find areas with this regulatory element")
      .def(
          "findUsages", +[](AreaLayer& self, ConstLineString3d& ls) { return self.findUsages(ls); }, arg("ls"),
          "Find areas with this linestring");
  wrapLayer<LaneletLayer, bases<PrimitiveLayer<Lanelet>>>("LaneletLayer")
      .def(
          "findUsages", +[](LaneletLayer& self, RegulatoryElementPtr& e) { return self.findUsages(e); }, arg("regelem"),
          "Find lanelets with this regualtory element")
      .def(
          "findUsages", +[](LaneletLayer& self, ConstLineString3d& ls) { return self.findUsages(ls); }, arg("ls"),
          "Lanelets with this linestring");
  wrapLayer<PolygonLayer>("PolygonLayer")
      .def(
          "findUsages", +[](PolygonLayer& self, ConstPoint3d& p) { return self.findUsages(p); }, arg("point"),
          "Find polygons with this point");
  wrapLayer<LineStringLayer>("LineStringLayer")
      .def(
          "findUsages", +[](LineStringLayer& self, ConstPoint3d& p) { return self.findUsages(p); }, arg("point"),
          "Find linestrings with this point");
  wrapLayer<PointLayer>("PointLayer");
  wrapLayer<RegulatoryElementLayer>("RegulatoryElementLayer");

  class_<LaneletMapLayers, boost::noncopyable>("LaneletMapLayers", "Container for the layers of a lanelet map")
      .def_readonly("laneletLayer", &LaneletMap::laneletLayer,
                    "Lanelets of this map (works like a dictionary with ID as key and Lanelet as value)")
      .def_readonly("areaLayer", &LaneletMap::areaLayer,
                    "Areas of this map (works like a dictionary with ID as key and Area as value)")
      .def_readonly(
          "regulatoryElementLayer", &LaneletMap::regulatoryElementLayer,
          "Regulatory elements of this map (works like a dictionary with ID as key and RegulatoryElement as value)")
      .def_readonly("lineStringLayer", &LaneletMap::lineStringLayer,
                    "Linestrings of this map (works like a dictionary with ID as key and Linestring3d as value)")
      .def_readonly("polygonLayer", &LaneletMap::polygonLayer,
                    "Polygons of this map (works like a dictionary with ID as key and Polygon3d as value)")
      .def_readonly("pointLayer", &LaneletMap::pointLayer,
                    "Points of this map (works like a dictionary with ID as key and Point3d as value)");

  class_<LaneletMap, bases<LaneletMapLayers>, LaneletMapPtr, boost::noncopyable>(
      "LaneletMap",
      "A lanelet map collects lanelet primitives. It can be used for writing and loading and creating routing "
      "graphs. "
      "It also offers geometrical and relational queries for its objects. Note that this is not the right object for "
      "querying neigborhood relations. Create a lanelet2.routing.RoutingGraph for this.\nNote that there is a "
      "utility "
      "function 'createMapFromX' to create a map from a list of primitives.",
      init<>("Create an empty lanelet map"))
      .def("add", selectAdd<Point3d>(), arg("point"),
           "Add a point to the map. It its ID is zero, it will be set to a unique value.")
      .def("add", selectAdd<Lanelet>(), arg("lanelet"),
           "Add a lanelet and all its subobjects to the map. It its (or its subobjects IDs) are zero, it will be set "
           "to a unique value.")
      .def("add", selectAdd<Area>(), arg("area"),
           "Add an area and all its subobjects to the map. It its (or its subobjects IDs) are zero, it will be set to "
           "a unique value.")
      .def("add", selectAdd<LineString3d>(), arg("linestring"),
           "Add a linestring and all its points to the map. It its (or its points IDs) are zero, it will be set to a "
           "unique value.")
      .def("add", selectAdd<Polygon3d>(), arg("polygon"),
           "Add a polygon and all its points to the map. It its (or its points IDs) are zero, it will be set to a "
           "unique value.")
      .def("add", selectAdd<const RegulatoryElementPtr&>(), arg("regelem"),
           "Add a regulatory element and all its subobjects to the map. It its (or its subobjects IDs) are zero, it "
           "will be set to a unique value.");
  register_ptr_to_python<LaneletMapConstPtr>();

  class_<LaneletSubmap, bases<LaneletMapLayers>, LaneletSubmapPtr, boost::noncopyable>(
      "LaneletSubmap",
      "A submap manages parts of a lanelet map. An important difference is that adding an object to the map will "
      "*not* "
      "add its subobjects too, making it more efficient to create. Apart from that, it offers a similar "
      "functionality "
      "compared to a lanelet map.",
      init<>("Create an empty lanelet submap"))
      .def(
          "laneletMap", +[](LaneletSubmap& self) { return LaneletMapPtr{self.laneletMap()}; },
          "Create a lanelet map of this submap that also holds all subobjects. This is a potentially costly "
          "operation.")
      .def("add", selectSubmapAdd<Point3d>(), arg("point"), "Add a point to the submap")
      .def("add", selectSubmapAdd<Lanelet>(), arg("lanelet"),
           "Add a lanelet to the submap, without its subobjects. If its ID is zero it will be set to a unique value "
           "instead.")
      .def("add", selectSubmapAdd<Area>(), arg("area"),
           "Add an area to the submap, without its subobjects. If its ID is zero it will be set to a unique value "
           "instead.")
      .def("add", selectSubmapAdd<LineString3d>(), arg("linestring"),
           "Add a linesting to the submap, without its points. If its ID is zero it will be set to a unique value "
           "instead.")
      .def("add", selectSubmapAdd<Polygon3d>(), arg("polygon"),
           "Add a polygon to the submap, without its points. If its ID is zero it will be set to a unique value "
           "instead.")
      .def("add", selectSubmapAdd<const RegulatoryElementPtr&>(), arg("regelem"),
           "Add a regulatory element to the submap, without its subobjects. If its ID is zero it will be set to a "
           "unique value "
           "instead.");
  register_ptr_to_python<LaneletSubmapConstPtr>();

  def("getId", static_cast<Id (&)()>(utils::getId), "Returns a globally unique id");
  def("registerId", &utils::registerId, "Registers an id (so that it will not be returned by getId)");

  def("createMapFromPoints", createMapWrapper<Points3d>, arg("points"), "Create map from a list of points");
  def("createMapFromLineStrings", createMapWrapper<LineStrings3d>, arg("linestrings"),
      "Create map from a list of linestrings");
  def("createMapFromPolygons", createMapWrapper<Polygons3d>, arg("polygons"), "Create map from a list of polygons");
  def("createMapFromLanelets", createMapWrapper<Lanelets>, arg("lanelets"), "Create map from a list of lanelets");
  def("createMapFromAreas", createMapWrapper<Areas>, arg("areas"), "Create map from a list of areas");

  def("createSubmapFromPoints", createSubmapWrapper<Points3d>, arg("points"), "Create sbumap from a list of points");
  def("createSubmapFromLineStrings", createSubmapWrapper<LineStrings3d>, arg("linestrings"),
      "Create submap from a list of linestrings");
  def("createSubmapFromPolygons", createSubmapWrapper<Polygons3d>, arg("polygons"),
      "Create submap from a list of polygons");
  def("createSubmapFromLanelets", createSubmapWrapper<Lanelets>, arg("lanelets"),
      "Create submap from a list of lanelets");
  def("createSubmapFromAreas", createSubmapWrapper<Areas>, arg("areas"), "Create submap from a list of areas");
}
