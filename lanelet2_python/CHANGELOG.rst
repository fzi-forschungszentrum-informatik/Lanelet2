^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Format files
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
