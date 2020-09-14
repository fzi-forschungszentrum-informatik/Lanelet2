^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
