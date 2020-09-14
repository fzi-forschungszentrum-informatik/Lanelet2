^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_io
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2020-09-14)
------------------

1.1.0 (2020-09-06)
------------------
* Add experimental support for building with colcon on ros2 and ament_cmake
* Format files with clang-format 10
* io: Fix random errors in bin serialization when serializing a HybridMap
  fixes fzi-forschungszentrum-informatik/Lanelet2#128
* Making all includes in lanelet2_io consistent.
* Updating package.xml files to format 3.
* Contributors: Fabian Poggenhans, Joshua Whitley

1.0.1 (2020-03-24)
------------------
* Make sure lanelet2 buildtool_export_depends on mrt_cmake_modules
* Add changelogs
* Improve warning if wrong decimal symbol is set, also report it when loading
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* Add a new class 'LaneletSubmap' that only contains parts of the map and is faster to construct
* IO: Implement warning for cases where the decimal point is overridden by a different locale
  resolves MRT/released/lanelet2#91
* Fix loading of polygons that have been written without an area tag
  resolves MRT/released/lanelet2#113
* Refactored osm parser so that parsed roles in relations keep their
  positions
* Improve c++17 support, supress warnings and clang-tidy 7 issues
* IO now complains when loading georeferenced maps with a default origin (resolves #71)
* Initial commit
* Contributors: Fabian Poggenhans, Maximilian Naumann
