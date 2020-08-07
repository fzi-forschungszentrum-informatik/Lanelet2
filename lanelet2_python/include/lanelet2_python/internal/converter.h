// FROM
// https://github.com/drodri/cppcon2016/blob/master/pythoncpp/boost/converter.h
#include <lanelet2_core/utility/Optional.h>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/indexing_suite.hpp>
#include <boost/variant/static_visitor.hpp>
//#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace wrappers {
namespace py = boost::python;
template <typename T>
auto getItem(T& obj, int64_t index) -> decltype(obj[0]) {
  if (index < 0) {
    index += obj.size();
  }
  if (index >= 0 && size_t(index) < obj.size()) {
    return obj[size_t(index)];
  }
  PyErr_SetString(PyExc_IndexError, "index out of range");
  py::throw_error_already_set();
  return obj[0];
}

template <typename T, typename ValT>
void setItem(T& obj, int64_t index, ValT value) {
  if (index < 0) {
    index += obj.size();
  }
  if (index >= 0 && size_t(index) < obj.size()) {
    obj[size_t(index)] = value;
    return;
  }
  PyErr_SetString(PyExc_IndexError, "index out of range");
  py::throw_error_already_set();
}

template <typename T>
void delItem(T& obj, int64_t index) {
  if (index < 0) {
    index += obj.size();
  }
  if (index >= 0 && size_t(index) < obj.size()) {
    obj.erase(std::next(obj.begin(), index));
    return;
  }
  PyErr_SetString(PyExc_IndexError, "index out of range");
  py::throw_error_already_set();
}
}  // namespace wrappers

namespace converters {
namespace py = boost::python;

struct IterableConverter {
  /// @note Registers converter from a python interable type to the
  ///       provided type.
  template <typename Container>
  IterableConverter& fromPython() {
    boost::python::converter::registry::push_back(
        &IterableConverter::convertible, &IterableConverter::construct<Container>, boost::python::type_id<Container>());

    // Support chaining.
    return *this;
  }

  /// @brief Check if PyObject is iterable.
  static void* convertible(PyObject* object) {
    auto* it = PyObject_GetIter(object);
    if (it != nullptr) {
      Py_DECREF(it);
      return object;
    }
    if (PyErr_ExceptionMatches(PyExc_TypeError) != 0) {
      PyErr_Clear();
    }
    return nullptr;
  }

  /// @brief Convert iterable PyObject to C++ container type.
  ///
  /// Container Concept requirements:
  ///
  ///   * Container::value_type is CopyConstructable.
  ///   * Container can be constructed and populated with two iterators.
  ///     I.e. Container(begin, end)
  template <typename Container>
  static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace python = boost::python;
    // Object is a borrowed reference, so create a handle indicting it is
    // borrowed for proper reference counting.
    python::handle<> handle(python::borrowed(object));

    // Obtain a handle to the memory block that the converter has allocated
    // for the C++ type.
    using StorageType = python::converter::rvalue_from_python_storage<Container>;
    void* storage = reinterpret_cast<StorageType*>(data)->storage.bytes;  // NOLINT

    using Iterator = python::stl_input_iterator<typename Container::value_type>;

    // Allocate the C++ type into the converter's memory block, and assign
    // its handle to the converter's convertible variable.  The C++
    // container is populated by passing the begin and end iterators of
    // the python object to the container's constructor.
    new (storage) Container(Iterator(python::object(handle)),  // begin
                            Iterator());                       // end
    data->convertible = storage;
  }
};
struct ToOptionalConverter {
  /// @note Registers converter from a python interable type to the
  ///       provided type.
  template <typename OptionalT>
  ToOptionalConverter& fromPython() {
    boost::python::converter::registry::push_back(&ToOptionalConverter::convertible,
                                                  &ToOptionalConverter::construct<OptionalT>,
                                                  boost::python::type_id<lanelet::Optional<OptionalT>>());
    // Support chaining.
    return *this;
  }

  /// @brief Check PyObject
  static void* convertible(PyObject* object) { return object; }

  /// @brief Convert PyObject to C++ type.
  template <typename OptionalT>
  static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace python = boost::python;
    using StorageType = python::converter::rvalue_from_python_storage<lanelet::Optional<OptionalT>>;
    void* storage = reinterpret_cast<StorageType*>(data)->storage.bytes;  // NOLINT

    if (object == Py_None) {
      new (storage) lanelet::Optional<OptionalT>();
    } else {
      new (storage) lanelet::Optional<OptionalT>(python::extract<OptionalT>(object));
    }
    data->convertible = storage;
  }
};

template <typename T>
struct VariantToObject : boost::static_visitor<PyObject*> {
  static result_type convert(const T& v) { return apply_visitor(VariantToObject(), v); }

  template <typename Type>
  result_type operator()(const Type& t) const {
    return boost::python::incref(boost::python::object(t).ptr());
  }
};

template <typename T>
struct VectorToList {
  static PyObject* convert(const T& v) {
    py::list l;
    for (auto& e : v) {
      l.append(e);
    }
    return py::incref(l.ptr());
  }
};

template <typename T>
struct OptionalToObject {
  static PyObject* convert(const lanelet::Optional<T>& v) {
    if (v) {
      return py::incref(py::object(v.get()).ptr());
    }
    // return none
    return py::incref(py::object().ptr());
  }
};

template <typename T>
struct WeakToObject {
  static PyObject* convert(const T& v) {
    if (!v.expired()) {
      return py::incref(py::object(v.lock()).ptr());
    }
    // return none
    return py::incref(py::object().ptr());
  }
};

template <typename T1, typename T2>
struct PairToPythonConverter {
  static PyObject* convert(const std::pair<T1, T2>& pair) {
    return py::incref(py::make_tuple(pair.first, pair.second).ptr());
  }
};

template <typename T1, typename T2>
struct PythonToPairConverter {
  PythonToPairConverter() {
    py::converter::registry::push_back(&convertible, &construct, py::type_id<std::pair<T1, T2>>());
  }
  static void* convertible(PyObject* obj) {
    if (!PyTuple_CheckExact(obj)) {
      return nullptr;
    }
    if (PyTuple_Size(obj) != 2) {
      return nullptr;
    }
    return obj;
  }
  static void construct(PyObject* obj, py::converter::rvalue_from_python_stage1_data* data) {
    py::tuple tuple(py::borrowed(obj));
    using StorageType = py::converter::rvalue_from_python_storage<std::pair<T1, T2>>;
    void* storage = reinterpret_cast<StorageType*>(data)->storage.bytes;  // NOLINT
    new (storage) std::pair<T1, T2>(py::extract<T1>(tuple[0])(), py::extract<T2>(tuple[1])());
    data->convertible = storage;
  }
};

template <typename T1, typename T2>
struct PyPair {
  py::to_python_converter<std::pair<T1, T2>, PairToPythonConverter<T1, T2>> toPy;
  PythonToPairConverter<T1, T2> fromPy;
};

template <typename T>
using VectorToListConverter = py::to_python_converter<T, VectorToList<T>>;

template <typename T>
using OptionalConverter = py::to_python_converter<lanelet::Optional<T>, OptionalToObject<T>>;

template <typename T>
using WeakConverter = py::to_python_converter<T, WeakToObject<T>>;

template <typename T>
using VariantConverter = py::to_python_converter<T, VariantToObject<T>>;

template <typename T>
using PairConverter = PyPair<typename T::first_type, typename T::second_type>;
}  // namespace converters
