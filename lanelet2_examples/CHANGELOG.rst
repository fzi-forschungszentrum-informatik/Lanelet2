^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2023-05-10)
------------------

1.2.0 (2023-01-30)
------------------
* Fix OSM file output for upload and elevation (`#245 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/245>`_)
  * discourage upload and format elevation to max 2 decimals to prevent JSOM excessive elevation precision errors
  * remove width to prevent leading spaces for lat/lon/ele
  * allow for providing parameters josm_prevent_upload and josm_format_elevation to write
  * Test lat/lon/ele formatting with and without josm_format_elevation. Test josm_upload
  * improve read/write tutorial section comments
  * document params
* Fix 223 all way stop python (`#231 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/231>`_)
* Add CI using GitHub Actions (`#256 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/256>`_)
* Add lanelet2_matching as dep for examples
* Move matching integration test to examples
* Update and fix conan build, deps and docs
  closes `fzi-forschungszentrum-informatik/Lanelet2#174 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/174>`_
* Fixed typo in README.md
* Contributors: Denzel-Haverkamp, Fabian Poggenhans, Maximilian Naumann, Michał Antkiewicz, Fabian Immel

1.1.1 (2020-09-14)
------------------

1.1.0 (2020-09-06)
------------------
* Add experimental support for building with colcon on ros2 and ament_cmake
* Make all includes of lanelet2_examples consistent
* Updating package.xml files to format 3.
* Apply clang-tidy 10 recommendations
* Contributors: Fabian Poggenhans, Joshua Whitley

1.0.1 (2020-03-24)
------------------
* Make sure lanelet2 buildtool_export_depends on mrt_cmake_modules
* Add changelogs
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* RoutingGraph and Route now use the new LaneletSubmap to store the lanelets they are using
  their member functions laneletMap() and passableMap() are now deprecated and should be replaced by laneletSubmap() and passableSubmap() respectively. These functions have less overhead and are less likely to be misinterpreted as 'maps containing only the lanelets you need'
* Add a new class 'LaneletSubmap' that only contains parts of the map and is faster to construct
* Routing graph and route object now support queries with a custom search function
* Improve c++17 support, supress warnings and clang-tidy 7 issues
* Simplified definition of origin in python, adapted examples
* Add examples package
* Contributors: Fabian Poggenhans, Maximilian Naumann, Sascha Wirges
