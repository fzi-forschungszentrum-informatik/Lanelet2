^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2023-05-10)
------------------
* Improve python core module (`#293 <https://github.com/immel-f/Lanelet2/issues/293>`_)
  Improve lanelet2.core python wrappers
  add docstrings, named arguments and __repr_\_ methods to core primitives in python, fix bugs and add more initialization options
  ---------
  Co-authored-by: Fabian Poggenhans <fabian.poggenhans@partner.kit.edu>
* Add readme to PyPi package description and fix readme icons (`#283 <https://github.com/immel-f/Lanelet2/issues/283>`_)
* Build lanelet2 wheel and publish in GH release and PyPI (`#278 <https://github.com/immel-f/Lanelet2/issues/278>`_)
* Contributors: Jan Rudolph, immel-f, poggenhans

1.2.0 (2023-01-30)
------------------
* Fix OSM file output for upload and elevation (`#245 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/245>`_)
  * discourage upload and format elevation to max 2 decimals to prevent JSOM excessive elevation precision errors
  * remove width to prevent leading spaces for lat/lon/ele
  * allow for providing parameters josm_prevent_upload and josm_format_elevation to write
  * Test lat/lon/ele formatting with and without josm_format_elevation. Test josm_upload
  * improve read/write tutorial section comments
  * document params
* add Geocentric and Local Cartesian projectors (`#244 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/244>`_)
* Fix 223 all way stop python (`#231 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/231>`_)
* Add CI using GitHub Actions (`#256 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/256>`_)
* Added pointer declaration for code quality issues
* Add a test case for GeometryApi to increase coverage
* Pass reference of const qualified parameter
* Add required boost::geometry functionalities to pyapi
* Add fromArcCoordinates() function to pyapi
* Python: Add basicPoint method to ConstPoint2d
  closes `fzi-forschungszentrum-informatik/Lanelet2#192 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/192>`_
* Switch to lanelet2.matching in python
* Move python bindings of lanelet2_matching to lanelet2_python
* Python: Fix find usages for const objects
  closes `fzi-forschungszentrum-informatik/lanelet2#168 <https://github.com/fzi-forschungszentrum-informatik/lanelet2/issues/168>`_
* Python api: fix getter of Area.outerBound
  closes `fzi-forschungszentrum-informatik/Lanelet2#152 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/152>`_
* Contributors: Fabian, Fabian Poggenhans, Frank Bieder, Maximilian Naumann, Michał Antkiewicz, Sahin Tas, bieder, Fabian Immel

1.1.1 (2020-09-14)
------------------

1.1.0 (2020-09-06)
------------------
* Add parameter to left/right/adjacentLeft/adjacentRight so that they can be queried based on routing cost id
* Add experimental support for building with colcon on ros2 and ament_cmake
* Format files with clang-format 10
* Add interpolatedPointAtDistance for BasicLineString
* Expose readwrite struct members to python
* Python functions for distance between CompoundLineStrings and LineStrings
* Python geometry interface for CompoundLineStrings
* Making all includes in lanelet2_python consistent.
* Add __hash__ for python bindings
* Add bindings for findWithin geometry function
* Updating package.xml files to format 3.
* Fix bindings for shortestPath function
* Fix memory leak in list->vector conversion
  closes fzi-forschungszentrum-informatik/Lanelet2#111
* SpeedLimitInformation now also offers m/s
* Fix const ptr issue in ConstLanelet.RightOfWay
* Contributors: Christian-Eike Framing, Fabian Poggenhans, Joshua Whitley, Maximilian Naumann

1.0.1 (2020-03-24)
------------------
* Fix python bindings for lanelet submap
* lanelet2_python: Register constructor for SpeedLimits
* Register more geometry functions (#96, #97)
* Register Lanelet::resetCache in python
* Make sure lanelet2 buildtool_export_depends on mrt_cmake_modules
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* Apply clang-tidy fixes
* lanelet2_python: move converter header to internal include dir
* added equals for points
* Python: Fix registration of LaneletMap::add(Point3d)
* lanelet2_python: Fix wrongly registered RoutingCostDistance
* Fix default values for lane changes in RoutingGraph
* RoutingGraph and Route now use the new LaneletSubmap to store the lanelets they are using
* Add a new class 'LaneletSubmap' that only contains parts of the map and is faster to construct
* Routing graph and route object now support queries with a custom search function
* Extended and simplified the reachablePath/Set functions
* Refactored the internal representation of the route. Cleaned up headers that are only supposed to be used internally
* Offer reverse routing (possibleRoutesTowards), bindings, unittests
* Refactor FilteredGraphs and RelationTypes to use bitmasks
* Improve the distance2d and distance3d to support generic distance computations
* Initial commit
* Contributors: Fabian Poggenhans, Johannes Janosovits, Maximilian Naumann
