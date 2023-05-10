^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2023-05-10)
------------------
* Update thirdparty deps, rework projected point, enable python 3.10/3.11 (`#300 <https://github.com/immel-f/Lanelet2/issues/300>`_)
* Fix typos in documentation (`#286 <https://github.com/immel-f/Lanelet2/issues/286>`_)
* Contributors: Johannes Schmitz, poggenhans

1.2.0 (2023-01-30)
------------------
* Add zero-width character after colon to avoid markdown emoji
  :car: :arrow_right: :&`#8203 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/8203>`_;car:
* Use TYPED_TEST_SUITE over deprecated TYPED_TEST_CASE in unit tests
* Add required boost::geometry functionalities to pyapi
* Fix bug with custom centerlines
  custom centerlines of lanelets are now properly added when creating a lanelet map. Added tests to ensure this
  fixes `fzi-forschungszentrum-informatik/Lanelet2#159 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/159>`_
* Remove randomness from test findNearestWorksForRandomLinestrings
* Fix build for boost 1.69
* Contributors: Fabian Poggenhans, Héctor Ochoa Ortiz, Maximilian Naumann, Sahin Tas, Fabian Immel

1.1.1 (2020-09-14)
------------------

1.1.0 (2020-09-06)
------------------
* Add experimental support for building with colcon on ros2 and ament_cmake
* Document undocumented parameter in determineCommonLine
* Fix an issue that findNearest returned more results than intended. Add test and clarify doc.
  fixes fzi-forschungszentrum-informatik/Lanelet2#142
* Python functions for distance between CompoundLineStrings and LineStrings
* Removing extra semicolons causing warnings with wpedantic.
* io: Fix random errors in bin serialization when serializing a HybridMap
  fixes fzi-forschungszentrum-informatik/Lanelet2#128
* Add missing std::hash specializations, add __hash__ for python bindings
* Making all includes in lanelet2_core consistent.
* Fix 2d/3d confusions in geometry functions and implemented proper errors. Extended distance3d interface
* Updating package.xml files to format 3.
* Improve and extend determineCommonLine utility function
* Add functionality to create the bounding polygon from a Path
* Remove useless regulatoryElementsAs template function from LineStringOrPolygon
* Apply clang-tidy 10 recommendations
* Contributors: Christian-Eike Framing, Fabian Poggenhans, Johannes Janosovits, Joshua Whitley, Yujie He, harderthan

1.0.1 (2020-03-24)
------------------
* Fix build failure if size_t is not unsigned long
* Fix build with boost 1.62
* Make sure lanelet2 buildtool_export_depends on mrt_cmake_modules
* Add geometry function to compute curvature from three points
  Add tests, cleanup the validator that uses it
* Add changelogs
* Format files
* Fix clang-tidy warnings
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* Add unittest for findWithin and areas
* Apply clang-tidy fixes
* Add geometry functions convexPartition and triangulate
  These handle splitting polygons into convex polygons (or into triangles)
* Improve documentation about Regulatory Elements
* Fix headers missing their include guards
* Fix CompoundLineString2d::rightBound2d()
* Add missing const in Area.h regulatoryElementsAs method
* Add a new class 'LaneletSubmap' that only contains parts of the map and is faster to construct
* Refactored the internal representation of the route. Cleaned up headers that are only supposed to be used internally
* Make sure HybridLineString returns const references instead of copies
* Fix comparison operators for laneletOrArea and LineStringOrPolygon
* Fix constness of search function in lanelet map
* Add a new geometry function that shifts line strings laterally
* Add US traffic signs to JOSM stylesheet and docu
* Add an AllWayStop regulatory element, tests and docs
* Initial commit
* Contributors: Fabian Poggenhans, Florian Kuhnt, Johannes Janosovits, Maximilian Naumann, Piotr Orzechowski
