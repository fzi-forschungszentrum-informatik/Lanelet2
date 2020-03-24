^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
