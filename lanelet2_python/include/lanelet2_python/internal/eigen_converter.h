// MIT License

// Copyright (c) 2019 Vincent SAMY

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <Eigen/Core>
#include <boost/python.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/numpy.hpp>

namespace py = boost::python;
namespace np = boost::python::numpy;

namespace converters {

/*
 *                  List -> Eigen converters
 */

/** Conversion from a python list to an Eigen Vector
 * This convert list of type T (int, float32, float64, ...) to an Eigen::VectorType
 * The template VectorType should be a type as Eigen::Matrix<T, rows, 1> or Eigen::Matrix<T, 1, cols>
 * Note it should also work for Eigen::Array
 */
template <class VectorType>
struct python_list_to_eigen_vector {
  python_list_to_eigen_vector() {
    py::converter::registry::push_back(&convertible, &construct, py::type_id<VectorType>());
  }

  static void* convertible(PyObject* obj_ptr) {
    static_assert(VectorType::ColsAtCompileTime == 1 || VectorType::RowsAtCompileTime == 1,
                  "Passed a Matrix into a Vector generator");  // Only for vectors conversion

    if (!PySequence_Check(obj_ptr) ||
        (VectorType::ColsAtCompileTime == 1 && VectorType::RowsAtCompileTime != Eigen::Dynamic &&
         (PySequence_Size(obj_ptr) != VectorType::RowsAtCompileTime))  // Check Fixed-size vector
        || (VectorType::RowsAtCompileTime == 1 && VectorType::ColsAtCompileTime != Eigen::Dynamic &&
            (PySequence_Size(obj_ptr) != VectorType::ColsAtCompileTime)))  // Check Fixed-size row vector
      return 0;

    if (VectorType::ColsAtCompileTime == 1 && VectorType::RowsAtCompileTime != Eigen::Dynamic &&
        (PySequence_Size(obj_ptr) != VectorType::RowsAtCompileTime))
      return 0;

    py::list arr = py::extract<py::list>(obj_ptr);
    for (long i = 0; i < py::len(arr); i++)
      if (!py::extract<typename VectorType::Scalar>(arr[i]).check()) return 0;

    return obj_ptr;
  }

  static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data) {
    py::list arr = py::extract<py::list>(obj_ptr);
    auto len = py::len(arr);

    using storage_type = py::converter::rvalue_from_python_storage<VectorType>;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    new (storage) VectorType;
    VectorType& vec = *static_cast<VectorType*>(storage);

    if (VectorType::RowsAtCompileTime == Eigen::Dynamic || VectorType::ColsAtCompileTime == Eigen::Dynamic)
      vec.resize(len);

    for (long i = 0; i < len; ++i) vec(i) = py::extract<typename VectorType::Scalar>(arr[i]);

    data->convertible = storage;
  }
};

/** Conversion from a python list to an Eigen Matrix
 * This convert list of list of type T (int, float32, float64, ...) to an Eigen::MatrixType
 * The template MatrixType should be a type as Eigen::Matrix<T, rows, cols>
 * Note it should also work for Eigen::Array
 */
template <class MatrixType>
struct python_list_to_eigen_matrix {
  python_list_to_eigen_matrix() {
    py::converter::registry::push_back(&convertible, &construct, py::type_id<MatrixType>());
  }

  static void* convertible(PyObject* obj_ptr) {
    static_assert(MatrixType::ColsAtCompileTime != 1 && MatrixType::RowsAtCompileTime != 1,
                  "Passed a Vector into a Matrix generator");  // Only for matrix conversion

    auto checkNestedList = [](const py::list& list) {
      py::extract<py::list> nested_list(list[0]);
      if (!nested_list.check()) return false;

      auto cols = py::len(nested_list());
      for (long i = 1; i < py::len(list); ++i) {
        py::extract<py::list> nested_list(list[i]);
        if (!nested_list.check() || py::len(nested_list) != cols)  // Check nested list size
          return false;
        for (long j = 0; j < cols; ++j)
          if (!py::extract<typename MatrixType::Scalar>(nested_list()[j]).check())  // Check list type
            return false;
      }

      return true;
    };

    py::extract<py::list> extract_list(obj_ptr);
    py::extract<py::list> extract_nested_list(extract_list());
    if (!extract_list.check()                // Check list
        || !checkNestedList(extract_list())  // Check nested lists
        || (MatrixType::RowsAtCompileTime != Eigen::Dynamic &&
            MatrixType::RowsAtCompileTime != py::len(extract_list()))  // Check rows are the same
        || (MatrixType::ColsAtCompileTime != Eigen::Dynamic &&
            MatrixType::ColsAtCompileTime != py::len(extract_nested_list())))  // Check cols are the same
      return 0;

    return obj_ptr;
  }

  static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data) {
    py::list arr = py::extract<py::list>(obj_ptr);
    auto rows = py::len(arr);
    auto cols = py::len(py::extract<py::list>(arr[0])());

    using storage_type = py::converter::rvalue_from_python_storage<MatrixType>;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    new (storage) MatrixType;
    MatrixType& mat = *static_cast<MatrixType*>(storage);

    // Resize matrix if (half-)dynamic-sized matrix
    if (MatrixType::RowsAtCompileTime == Eigen::Dynamic || MatrixType::ColsAtCompileTime == Eigen::Dynamic)
      mat.resize(rows, cols);

    for (long i = 0; i < rows; ++i)
      for (long j = 0; j < cols; ++j) mat(i, j) = py::extract<typename MatrixType::Scalar>(arr[i][j]);

    data->convertible = storage;
  }
};

/*
 *              Numpy -> Eigen converters
 */

/** Conversion from a numpy ndarray to an Eigen Vector
 * This convert numpy.array of dtype T (int, float32, float64, ...) to an Eigen::VectorType
 * The template VectorType should be a type as Eigen::Matrix<T, rows, 1> or Eigen::Matrix<T, 1, cols>
 * Note it should also work for Eigen::Array
 */
template <typename VectorType>
struct numpy_array_to_eigen_vector {
  numpy_array_to_eigen_vector() {
    py::converter::registry::push_back(&convertible, &construct, py::type_id<VectorType>());
  }

  static void* convertible(PyObject* obj_ptr) {
    static_assert(VectorType::RowsAtCompileTime == 1 || VectorType::ColsAtCompileTime == 1,
                  "Passed a Matrix into a Vector generator");  // Check that in c++ side, it is an Eigen vector
    py::extract<np::ndarray> arr(obj_ptr);
    if (!arr.check()  // Check it is a numpy array
        || arr().get_nd() !=
               1  // Check array dimension (does not allow numpy array of type (1, 3), needs to ravel it first)
        || arr().get_dtype() != np::dtype::get_builtin<typename VectorType::Scalar>()  // Check type
        || (VectorType::RowsAtCompileTime == 1 && VectorType::ColsAtCompileTime != Eigen::Dynamic &&
            VectorType::ColsAtCompileTime !=
                arr().shape(0))  // Check vector size in case of fixed-size array (for a row-vector)
        || (VectorType::ColsAtCompileTime == 1 && VectorType::RowsAtCompileTime != Eigen::Dynamic &&
            VectorType::RowsAtCompileTime !=
                arr().shape(0)))  // Check vector size in case of fixed-size array (for a column-vector)
      return 0;

    return obj_ptr;
  }

  static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data) {
    np::ndarray arr = py::extract<np::ndarray>(obj_ptr);

    using storage_type = py::converter::rvalue_from_python_storage<VectorType>;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    new (storage) VectorType;
    VectorType& vec = *static_cast<VectorType*>(storage);
    // Resize for dynamic-sized matrices
    if (VectorType::RowsAtCompileTime == Eigen::Dynamic || VectorType::ColsAtCompileTime == Eigen::Dynamic)
      vec.resize(arr.shape(0));

    // Extract values. The type has been check in the convertible function
    for (int i = 0; i < arr.shape(0); ++i) vec(i) = py::extract<typename VectorType::Scalar>(arr[i]);

    data->convertible = storage;
  }
};

/** Conversion from a numpy ndarray to an Eigen Matrix
 * This convert numpy.array of dtype T (int, float32, float64, ...) to an Eigen::MatrixType
 * The template MatrixType should be a type as Eigen::Matrix<T, rows, cols>
 * Note it should also work for Eigen::Array
 */
template <typename MatrixType>
struct numpy_array_to_eigen_matrix {
  numpy_array_to_eigen_matrix() {
    py::converter::registry::push_back(&convertible, &construct, py::type_id<MatrixType>());
  }

  static void* convertible(PyObject* obj_ptr) {
    static_assert(MatrixType::ColsAtCompileTime != 1 && MatrixType::RowsAtCompileTime != 1,
                  "Passed a Vector into a Matrix generator");  // Only for matrix conversion

    py::extract<np::ndarray> arr(obj_ptr);
    if (!arr.check()                                                                   // Check it is a numpy array
        || arr().get_nd() != 2                                                         // Check array dimension
        || arr().get_dtype() != np::dtype::get_builtin<typename MatrixType::Scalar>()  // Check type
        || (MatrixType::RowsAtCompileTime != Eigen::Dynamic &&
            MatrixType::RowsAtCompileTime != arr().shape(0))  // Check rows are the same
        || (MatrixType::ColsAtCompileTime != Eigen::Dynamic &&
            MatrixType::ColsAtCompileTime != arr().shape(1)))  // Check cols are the same
      return 0;

    return obj_ptr;
  }

  static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data) {
    np::ndarray arr = py::extract<np::ndarray>(obj_ptr);

    using storage_type = py::converter::rvalue_from_python_storage<MatrixType>;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    new (storage) MatrixType;
    MatrixType& mat = *static_cast<MatrixType*>(storage);
    // Resize for dynamic-sized matrices
    // For half dynamic-sized matrices, the fixed-size part has been check in the convertible function
    if (MatrixType::RowsAtCompileTime == Eigen::Dynamic || MatrixType::ColsAtCompileTime == Eigen::Dynamic)
      mat.resize(arr.shape(0), arr.shape(1));

    // Extract values. The type has been check in the convertible function
    for (int i = 0; i < arr.shape(0); ++i)
      for (int j = 0; j < arr.shape(1); ++j) mat(i, j) = py::extract<typename MatrixType::Scalar>(arr[i][j]);

    data->convertible = storage;
  }
};

/*
 *              Eigen -> Numpy converters
 */

/** Conversion from an Eigen Vector to an numpy ndarray
 * This convert Eigen::VectorType of type T (int, float, double, ...) to an numpy.array
 * The template VectorType should be a type as Eigen::Matrix<T, rows, 1> or Eigen::Matrix<T, 1, cols>
 * Note it should also work for Eigen::Array
 */
template <typename VectorType>
struct eigen_vector_to_numpy_array {
  static PyObject* convert(const VectorType& mat) {
    static_assert(VectorType::ColsAtCompileTime == 1 || VectorType::RowsAtCompileTime == 1,
                  "Passed a Matrix into a Vector generator");  // Ensure that it is a vector

    np::dtype dt = np::dtype::get_builtin<typename VectorType::Scalar>();
    auto shape = py::make_tuple(mat.size());
    np::ndarray mOut = np::empty(shape, dt);

    for (Eigen::Index i = 0; i < mat.size(); ++i) mOut[i] = mat(i);

    return py::incref(mOut.ptr());
  }
};

/** Conversion from an Eigen Matrix to an numpy ndarray
 * This convert Eigen::MatrixType of type T (int, float, double, ...) to an numpy.array
 * The template MatrixType should be a type as Eigen::Matrix<T, rows, cols>
 * Note it should also work for Eigen::Array
 */
template <typename MatrixType>
struct eigen_matrix_to_numpy_array {
  static PyObject* convert(const MatrixType& mat) {
    static_assert(MatrixType::ColsAtCompileTime != 1 && MatrixType::RowsAtCompileTime != 1,
                  "Passed a Vector into a Matrix generator");  // Ensure that it is not a vector

    np::dtype dt = np::dtype::get_builtin<typename MatrixType::Scalar>();
    auto shape = py::make_tuple(mat.rows(), mat.cols());
    np::ndarray mOut = np::empty(shape, dt);

    for (Eigen::Index i = 0; i < mat.rows(); ++i)
      for (Eigen::Index j = 0; j < mat.cols(); ++j) mOut[i][j] = mat(i, j);

    return py::incref(mOut.ptr());
  }
};

/*
 * Main converters
 */

/** Main converters
 * Simple enum struct for simplification of common Eigen global typedefs
 */
enum struct Converters : short {
  None = 0,             /*!< No conversion */
  Matrix = 1,           /*!< Convert all global typedefs matrices */
  Vector = 1 << 1,      /*!< Convert all global typedefs vectors */
  RowVector = 1 << 2,   /*!< Convert all global typedefs row vectors */
  Array = 1 << 3,       /*!< Convert all global typedefs arrays */
  ColumnArray = 1 << 4, /*!< Convert all global typedefs column arrays */
  RowArray = 1 << 5,    /*!< Convert all global typedefs row arrays */
  NoStandard = 1 << 6,  /*! Convert some non standard yet usually used bindings (Vector6, Matrix6, Array6) */

  NoRowMatrixConversion = Matrix | Vector, /*!< Convert all global typedefs matrices and column vectors */
  AllMatrixConversion =
      Matrix | Vector | RowVector,            /*!< Convert all global typedefs matrices and (column and row) vectors */
  NoRowArrayConversion = Array | ColumnArray, /*!< Convert all global typedefs arrays and column arrays */
  AllArrayConversion =
      Array | ColumnArray | RowArray, /*!< Convert all global typedefs arrays and (column and row) vectors */
  All = AllMatrixConversion | AllArrayConversion | NoStandard /*!< Convert everything */
};

inline short operator&(Converters lhs, Converters rhs) { return static_cast<short>(lhs) & static_cast<short>(rhs); }

/** Global matrix conversion
 * Generate the conversion for Eigen global matrices typedefs, depending of the type T (int, float, double, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen Matrix
 */
template <typename T>
void convertAllMatrix(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, 2, 2> >();                            // Matrix2<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, 2> >();               // MatrixX2<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, 2, Eigen::Dynamic> >();               // Matrix2X<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, 3, 3> >();                            // Matrix3<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, 3> >();               // MatrixX3<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, 3, Eigen::Dynamic> >();               // Matrix3X<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, 4, 4> >();                            // Matrix4<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, 4> >();               // MatrixX4<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, 4, Eigen::Dynamic> >();               // Matrix4X<T>
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> >();  // MatrixX<T>

  // list -> eigen
  if (isListConvertible) {
    python_list_to_eigen_matrix<Eigen::Matrix<T, 2, 2> >();                            // Matrix2<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, 2> >();               // MatrixX2<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, 2, Eigen::Dynamic> >();               // Matrix2X<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, 3, 3> >();                            // Matrix3<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, 3> >();               // MatrixX3<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, 3, Eigen::Dynamic> >();               // Matrix3X<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, 4, 4> >();                            // Matrix4<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, 4> >();               // MatrixX4<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, 4, Eigen::Dynamic> >();               // Matrix4X<T>
    python_list_to_eigen_matrix<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> >();  // MatrixX<T>
  }

  // eigen -> python
  py::to_python_converter<Eigen::Matrix<T, 2, 2>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, 2, 2> > >();  // Matrix2<T>
  py::to_python_converter<Eigen::Matrix<T, Eigen::Dynamic, 2>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, Eigen::Dynamic, 2> > >();  // MatrixX2<T>
  py::to_python_converter<Eigen::Matrix<T, 2, Eigen::Dynamic>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, 2, Eigen::Dynamic> > >();  // Matrix2X<T>
  py::to_python_converter<Eigen::Matrix<T, 3, 3>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, 3, 3> > >();  // Matrix3<T>
  py::to_python_converter<Eigen::Matrix<T, Eigen::Dynamic, 3>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, Eigen::Dynamic, 3> > >();  // MatrixX3<T>
  py::to_python_converter<Eigen::Matrix<T, 3, Eigen::Dynamic>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, 3, Eigen::Dynamic> > >();  // Matrix3X<T>
  py::to_python_converter<Eigen::Matrix<T, 4, 4>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, 4, 4> > >();  // Matrix4<T>
  py::to_python_converter<Eigen::Matrix<T, Eigen::Dynamic, 4>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, Eigen::Dynamic, 4> > >();  // MatrixX4<T>
  py::to_python_converter<Eigen::Matrix<T, 4, Eigen::Dynamic>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, 4, Eigen::Dynamic> > >();  // Matrix4X<T>
  py::to_python_converter<
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
      eigen_matrix_to_numpy_array<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > >();  // MatrixX<T>
}

/** Global column vector conversion
 * Generate the conversion for Eigen global column vectors typedefs, depending of the type T (int, float, double, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen Vector
 */
template <typename T>
void convertAllVector(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 2, 1> >();               // Vector2<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 3, 1> >();               // Vector3<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 4, 1> >();               // Vector4<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, Eigen::Dynamic, 1> >();  // VectorX<T>

  // list -> eigen
  if (isListConvertible) {
    python_list_to_eigen_vector<Eigen::Matrix<T, 2, 1> >();               // Vector2<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, 3, 1> >();               // Vector3<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, 4, 1> >();               // Vector4<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, Eigen::Dynamic, 1> >();  // VectorX<T>
  }

  // eigen -> python
  py::to_python_converter<Eigen::Matrix<T, 2, 1>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 2, 1> > >();  // Vector2<T>
  py::to_python_converter<Eigen::Matrix<T, 3, 1>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 3, 1> > >();  // Vector3<T>
  py::to_python_converter<Eigen::Matrix<T, 4, 1>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 4, 1> > >();  // Vector4<T>
  py::to_python_converter<Eigen::Matrix<T, Eigen::Dynamic, 1>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, Eigen::Dynamic, 1> > >();  // VectorX<T>
}

/** Global row vector conversion
 * Generate the conversion for Eigen global row vectors typedefs, depending of the type T (int, float, double, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen RowVector
 */
template <typename T>
void convertAllRowVector(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 1, 2> >();               // RowVector2<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 1, 3> >();               // RowVector3<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 1, 4> >();               // RowVector4<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 1, Eigen::Dynamic> >();  // RowVectorX<T>

  // list -> eigen
  if (isListConvertible) {
    python_list_to_eigen_vector<Eigen::Matrix<T, 1, 2> >();               // RowVector2<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, 1, 3> >();               // RowVector3<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, 1, 4> >();               // RowVector4<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, 1, Eigen::Dynamic> >();  // RowVectorX<T>
  }

  // eigen -> python
  py::to_python_converter<Eigen::Matrix<T, 1, 2>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 1, 2> > >();  // RowVector2<T>
  py::to_python_converter<Eigen::Matrix<T, 1, 3>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 1, 3> > >();  // RowVector3<T>
  py::to_python_converter<Eigen::Matrix<T, 1, 4>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 1, 4> > >();  // RowVector4<T>
  py::to_python_converter<Eigen::Matrix<T, 1, Eigen::Dynamic>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 1, Eigen::Dynamic> > >();  // RowVectorX<T>
}

/** Global array conversion
 * Generate the conversion for Eigen global arrays typedefs, depending of the type T (int, float, double, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen Array
 */
template <typename T>
void convertAllArray(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_matrix<Eigen::Array<T, 2, 2> >();                            // Array2<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, 2> >();               // ArrayX2<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, 2, Eigen::Dynamic> >();               // Array2X<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, 3, 3> >();                            // Array3<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, 3> >();               // ArrayX3<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, 3, Eigen::Dynamic> >();               // Array3X<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, 4, 4> >();                            // Array4<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, 4> >();               // ArrayX4<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, 4, Eigen::Dynamic> >();               // Array4X<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic> >();  // ArrayX<T>

  // list -> eigen
  if (isListConvertible) {
    python_list_to_eigen_matrix<Eigen::Array<T, 2, 2> >();                            // Array2<T>
    python_list_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, 2> >();               // ArrayX2<T>
    python_list_to_eigen_matrix<Eigen::Array<T, 2, Eigen::Dynamic> >();               // Array2X<T>
    python_list_to_eigen_matrix<Eigen::Array<T, 3, 3> >();                            // Array3<T>
    python_list_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, 3> >();               // ArrayX3<T>
    python_list_to_eigen_matrix<Eigen::Array<T, 3, Eigen::Dynamic> >();               // Array3X<T>
    python_list_to_eigen_matrix<Eigen::Array<T, 4, 4> >();                            // Array4<T>
    python_list_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, 4> >();               // ArrayX4<T>
    python_list_to_eigen_matrix<Eigen::Array<T, 4, Eigen::Dynamic> >();               // Array4X<T>
    python_list_to_eigen_matrix<Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic> >();  // ArrayX<T>
  }

  // eigen -> python
  py::to_python_converter<Eigen::Array<T, 2, 2>, eigen_matrix_to_numpy_array<Eigen::Array<T, 2, 2> > >();  // Array2<T>
  py::to_python_converter<Eigen::Array<T, Eigen::Dynamic, 2>,
                          eigen_matrix_to_numpy_array<Eigen::Array<T, Eigen::Dynamic, 2> > >();  // ArrayX2<T>
  py::to_python_converter<Eigen::Array<T, 2, Eigen::Dynamic>,
                          eigen_matrix_to_numpy_array<Eigen::Array<T, 2, Eigen::Dynamic> > >();            // Array2X<T>
  py::to_python_converter<Eigen::Array<T, 3, 3>, eigen_matrix_to_numpy_array<Eigen::Array<T, 3, 3> > >();  // Array3<T>
  py::to_python_converter<Eigen::Array<T, Eigen::Dynamic, 3>,
                          eigen_matrix_to_numpy_array<Eigen::Array<T, Eigen::Dynamic, 3> > >();  // ArrayX3<T>
  py::to_python_converter<Eigen::Array<T, 3, Eigen::Dynamic>,
                          eigen_matrix_to_numpy_array<Eigen::Array<T, 3, Eigen::Dynamic> > >();            // Array3X<T>
  py::to_python_converter<Eigen::Array<T, 4, 4>, eigen_matrix_to_numpy_array<Eigen::Array<T, 4, 4> > >();  // Array4<T>
  py::to_python_converter<Eigen::Array<T, Eigen::Dynamic, 4>,
                          eigen_matrix_to_numpy_array<Eigen::Array<T, Eigen::Dynamic, 4> > >();  // ArrayX4<T>
  py::to_python_converter<Eigen::Array<T, 4, Eigen::Dynamic>,
                          eigen_matrix_to_numpy_array<Eigen::Array<T, 4, Eigen::Dynamic> > >();  // Array4X<T>
  py::to_python_converter<
      Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic>,
      eigen_matrix_to_numpy_array<Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic> > >();  // ArrayX<T>
}

/** Global column array conversion
 * Generate the conversion for Eigen global column arrays typedefs, depending of the type T (int, float, double, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen Array
 */
template <typename T>
void convertAllColumnArray(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_vector<Eigen::Array<T, 2, 1> >();               // Vector2<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, 3, 1> >();               // Vector3<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, 4, 1> >();               // Vector4<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, Eigen::Dynamic, 1> >();  // VectorX<T>

  // list -> eigen
  if (isListConvertible) {
    python_list_to_eigen_vector<Eigen::Array<T, 2, 1> >();               // Vector2<T>
    python_list_to_eigen_vector<Eigen::Array<T, 3, 1> >();               // Vector3<T>
    python_list_to_eigen_vector<Eigen::Array<T, 4, 1> >();               // Vector4<T>
    python_list_to_eigen_vector<Eigen::Array<T, Eigen::Dynamic, 1> >();  // VectorX<T>
  }

  // eigen -> python
  py::to_python_converter<Eigen::Array<T, 2, 1>, eigen_vector_to_numpy_array<Eigen::Array<T, 2, 1> > >();  // Vector2<T>
  py::to_python_converter<Eigen::Array<T, 3, 1>, eigen_vector_to_numpy_array<Eigen::Array<T, 3, 1> > >();  // Vector3<T>
  py::to_python_converter<Eigen::Array<T, 4, 1>, eigen_vector_to_numpy_array<Eigen::Array<T, 4, 1> > >();  // Vector4<T>
  py::to_python_converter<Eigen::Array<T, Eigen::Dynamic, 1>,
                          eigen_vector_to_numpy_array<Eigen::Array<T, Eigen::Dynamic, 1> > >();  // VectorX<T>
}

/** Global row array conversion
 * Generate the conversion for Eigen global row arrays typedefs, depending of the type T (int, float, double, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen Array
 */
template <typename T>
void convertAllRowArray(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_vector<Eigen::Array<T, 1, 2> >();               // RowVector2<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, 1, 3> >();               // RowVector3<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, 1, 4> >();               // RowVector4<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, 1, Eigen::Dynamic> >();  // RowVectorX<T>

  // list -> eigen
  if (isListConvertible) {
    python_list_to_eigen_vector<Eigen::Array<T, 1, 2> >();               // RowVector2<T>
    python_list_to_eigen_vector<Eigen::Array<T, 1, 3> >();               // RowVector3<T>
    python_list_to_eigen_vector<Eigen::Array<T, 1, 4> >();               // RowVector4<T>
    python_list_to_eigen_vector<Eigen::Array<T, 1, Eigen::Dynamic> >();  // RowVectorX<T>
  }

  // eigen -> python
  py::to_python_converter<Eigen::Array<T, 1, 2>,
                          eigen_vector_to_numpy_array<Eigen::Array<T, 1, 2> > >();  // RowVector2<T>
  py::to_python_converter<Eigen::Array<T, 1, 3>,
                          eigen_vector_to_numpy_array<Eigen::Array<T, 1, 3> > >();  // RowVector3<T>
  py::to_python_converter<Eigen::Array<T, 1, 4>,
                          eigen_vector_to_numpy_array<Eigen::Array<T, 1, 4> > >();  // RowVector4<T>
  py::to_python_converter<Eigen::Array<T, 1, Eigen::Dynamic>,
                          eigen_vector_to_numpy_array<Eigen::Array<T, 1, Eigen::Dynamic> > >();  // RowVectorX<T>
}

/** No standard conversion
 * Generate the conversion for no standard Eigen matrix and vector, depending of the type T (int, float, double, ...)
 * Those are Matrix6, Vector6, RowVector6 and Array6
 * \param isListConvertible if true, generate conversion from python list to Eigen Matrix
 */
template <typename T>
void convertNoStandard(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_matrix<Eigen::Matrix<T, 6, 6> >();  // Matrix6<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 6, 1> >();  // Vector6<T>
  numpy_array_to_eigen_vector<Eigen::Matrix<T, 1, 6> >();  // RowVector6<T>
  numpy_array_to_eigen_matrix<Eigen::Array<T, 6, 6> >();   // Array6<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, 6, 1> >();   // ColumnArray6<T>
  numpy_array_to_eigen_vector<Eigen::Array<T, 1, 6> >();   // RowArray6<T>

  // list -> eigen
  if (isListConvertible) {
    python_list_to_eigen_matrix<Eigen::Matrix<T, 6, 6> >();  // Matrix6<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, 6, 1> >();  // Vector6<T>
    python_list_to_eigen_vector<Eigen::Matrix<T, 1, 6> >();  // RowVector6<T>
    python_list_to_eigen_matrix<Eigen::Array<T, 6, 6> >();   // Array6<T>
    python_list_to_eigen_vector<Eigen::Array<T, 6, 1> >();   // ColumnArray6<T>
    python_list_to_eigen_vector<Eigen::Array<T, 1, 6> >();   // RowArray6<T>
  }

  // eigen -> python
  py::to_python_converter<Eigen::Matrix<T, 6, 6>,
                          eigen_matrix_to_numpy_array<Eigen::Matrix<T, 6, 6> > >();  // Matrix6<T>
  py::to_python_converter<Eigen::Matrix<T, 6, 1>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 6, 1> > >();  // Vector6<T>
  py::to_python_converter<Eigen::Matrix<T, 1, 6>,
                          eigen_vector_to_numpy_array<Eigen::Matrix<T, 1, 6> > >();  // RowVector6<T>
  py::to_python_converter<Eigen::Array<T, 6, 6>, eigen_matrix_to_numpy_array<Eigen::Array<T, 6, 6> > >();  // Array6<T>
  py::to_python_converter<Eigen::Array<T, 6, 1>,
                          eigen_vector_to_numpy_array<Eigen::Array<T, 6, 1> > >();  // ColumnArray6<T>
  py::to_python_converter<Eigen::Array<T, 1, 6>,
                          eigen_vector_to_numpy_array<Eigen::Array<T, 1, 6> > >();  // RowArray6<T>
}

/** Eigen Matrix and Array conversion
 * Generate the conversion for Eigen MatrixType (MatrixXd, Array2Xf, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen MatrixType
 */
template <typename MatrixType>
void convertMatrix(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_matrix<MatrixType>();

  // list -> eigen
  if (isListConvertible) python_list_to_eigen_matrix<MatrixType>();

  // eigen -> python
  py::to_python_converter<MatrixType, eigen_matrix_to_numpy_array<MatrixType> >();  // Vector2<T>
}

/** Eigen Vector and Array conversion
 * Generate the conversion for Eigen VectorType (VectorXd, RowVector4f, ArrayXf, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen VectorType
 */
template <typename VectorType>
void convertVector(bool isListConvertible = true) {
  // python -> eigen
  numpy_array_to_eigen_vector<VectorType>();

  // list -> eigen
  if (isListConvertible) python_list_to_eigen_vector<VectorType>();

  // eigen -> python
  py::to_python_converter<VectorType, eigen_vector_to_numpy_array<VectorType> >();
}

/** Helper functions to generate the converters
 * Generate all desired converters automatically
 * \param converters A bitFlag that tells the function what to convert. \see Converters
 * \param isListConvertible if true, generate conversion from python list to Eigen RowVector
 */
template <typename T>
void convert(Converters converters, bool isListConvertible = true) {
  if (converters & Converters::Matrix) convertAllMatrix<T>(isListConvertible);
  if (converters & Converters::Vector) convertAllVector<T>(isListConvertible);
  if (converters & Converters::RowVector) convertAllRowVector<T>(isListConvertible);
  if (converters & Converters::Array) convertAllArray<T>(isListConvertible);
  if (converters & Converters::ColumnArray) convertAllColumnArray<T>(isListConvertible);
  if (converters & Converters::RowArray) convertAllRowArray<T>(isListConvertible);
  if (converters & Converters::NoStandard) convertNoStandard<T>(isListConvertible);
}

}  // namespace converters