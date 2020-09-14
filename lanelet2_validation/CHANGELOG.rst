^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_validation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2020-09-14)
------------------

1.1.0 (2020-09-06)
------------------
* Add experimental support for building with colcon on ros2 and ament_cmake
* Making all includes in lanelet2_validation consistent.
* Updating package.xml files to format 3.
* Contributors: Fabian Poggenhans, Joshua Whitley

1.0.1 (2020-03-24)
------------------
* Make sure lanelet2 buildtool_export_depends on mrt_cmake_modules
* Add geometry function to compute curvature from three points
  Add tests, cleanup the validator that uses it
* Add changelogs
* Format files
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* Update cmakelists files
* Fix headers missing their include guards
* Bump version to 0.9
* Merge branch 'master' into reverse_routing
* Merge branch 'validate_repeated_points' into 'master'
  Validate repeated points
  See merge request MRT/released/lanelet2!142
* Add a new IssueReport object returned by lanelet2_validation
* Implement new check in lanelet2_validation for repeated points
* code clean, fix one big curvature point in mapping_example.osm
* add curvature validator and unit test
* Improved readmes and corrected typos
* Documented ele tag for points, add it to validation
* Re-add support for hardcoding sign type or speed limit in regulatory elements
* Add new validators to sanitize tagging of a lanelet map, add unittests
* Add lanelet2 validation
* Contributors: Fabian Poggenhans, Lingguang Wang, Maximilian Naumann
