# Geometry Calculations With Lanelet2

Lanelet2 primitives interface with [Boost.Geometry](https://github.com/boostorg/geometry). Boost.Geometry offers almost all common geometry calculations and is very fast. One downside is that not all algorithms work well with normal Lanelet2 primitives (see below). Another downside is that Boost.Geometry is compile-time heavy and is thus not included in the normal lanelet2 headers. To use geometry calculations, include the respective geometry header, e.g. `geometry/LineString.h`. 

Thanks to boost, all common geometry algorithms are available out of the box. E.g. you can compute distances between points, linestrings, polygons, etc in all combinations in (mostly) all dimensions.

There are usually two different kinds of algorithms: The one that Boost implements (like `distance`) and the one that Lanelet2 implements (mostly on top of Boost), like `boundingBox2d`. When using the first kind, you should read the lines below, while the second kind can be used without further reading.

For a list of algorithms that are available, please refer too Boost's documentation or look through lanelet's geometry headers (or doxygen). All algorithms there have a small description.

## Using Lanelet Primitives in Boost.Geometry

Lanelet2 offers every geometrical primitive in three flavors, each for 2D and 3D. Because they are just pointers to the actual data, they can be converted without actually copying data:
* **Mutable** (e.g. `LineString2d`): Are mutable, returned members are mutable (unless the object is const, then they are also immutable)
* **Const** (e.g. `ConstLineString2d`): These are immutable, returned members (e.g. points of linestring) are also immutable
* **Hybrid** (e.g. `ConstHybridLineString2d`): Also immutable, returned members are not lanelet primitives (e.g. `BasicPoint2d`). *If in doubt, use this one*.

Let us consider these types one by one. The first one (**mutable**) has the property that it behaves differently (i.e. returns different types) when used `const` or `non-const`. This is an issue for some of Boost.Geometry algorithms, because they sometimes accept const and sometimes non-const objects and therefore get the type wrong. Even if they get the type right, Boost is not fully compatible with the concept that copied primitives still refer to the same data and might therefore accidentally modify the wrong data. Therefore algorithms that modify the input (e.g. `correct`) are possible, but there is no 100% guarantee they work as expected (across all versions of Boost).

The **const** version does not have the risk of accidentally modifying the wrong data (because they are always immutable), however some algorithms that should not modify the data still try to instanciate templates in which the data *can* be modified. This results in longish compiler errors (and this might change from Boost version to Boost version). Algorithms that modify the input data (such as `correct`) can not be used because of the constness.

The **hybrid** version returns non-lanelet objects (BasicPoint2d/3d), which are fully compatible with Boost. This is the best solution for almost all geometry calculations. However, algorithms that mutate the primitive itself (such as `correct`) are not possible because the hybrid versions themselves are immutable (no points can be deleted or added).

In summary:

| Type    | For example             | Use for Boost.Geometry                          |
|---------|-------------------------|-------------------------------------------------|
| Mutable | LineString2d            | Only for mutating algorithms, but use with care |
| Const   | ConstLineString2d       | No                                              |
| Hybrid  | ConstHybridLineString2d | Yes, safe to use if they compile                |

## Understanding Boost Geometry's Errors

Boost geometry is known for outputting endless lines of compiler errors when used in the wrong way. Here are some hints to find out what you did wrong (sometimes you have to look closely for the actual error in many lines of instanciated templates). They are related to GCC's error messages, but other compilers will output similar stuff:
* Something about "no member named 'set' in boost::geometry::traits::access [...]: You used a const primitive (or you used a mutable primitive and Boost converted it into a const by a mistake). Try using the hybrid version.
* Some error including "NOT_IMPLEMENTED_FOR_THIS_POINT_TYPE" together with some *no matching function for call to assertion failed": This is a very generic error and the error message may be misleading. One reason could be that you forgot to include some `lanelet2_core/geometry` headers. Other reasons could be that you used the function on a primitive it was not implemented for (refer to Boosts documentation for that) or that it was not implemented for this particular dimension. Especially 3D operations are often not implemented in boost::geometry.
* Something with "You mixed matrices of different sizes". This is actually an error from `Eigen`. It means you passed a `BasicPoint3d` where a `BasicPoint2d` was expected (or vice versa).
* Something with "no matching member function for call to '_init1'", also from `Eigen`: You passed a wrong type where a BasicPoint2d/3d was expected.
* Some error in `boost::assert_dimension_equal`: You passed a 2d primitive to Boost where a 3d primitive was expected (or vice versa).
