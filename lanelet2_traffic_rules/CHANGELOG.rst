^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_traffic_rules
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2023-05-10)
------------------

1.2.0 (2023-01-30)
------------------
* Add signs for German zone 30 and zone 20 (`#264 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/264>`_)
* Add CI using GitHub Actions (`#256 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/256>`_)
* fix bug with explicitly defined lange change tags
* Fixed TrafficRulesFactory ignoring exact registry matches in case of vehicle subtypes (closes `#202 <https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/202>`_)
* Contributors: Maximilian Naumann, Patrick Peltzer, Fabian Immel, mitsudome-r

1.1.1 (2020-09-14)
------------------

1.1.0 (2020-09-06)
------------------
* Add experimental support for building with colcon on ros2 and ament_cmake
* Reorder includes with clang-format
* Making all includes in lanelet2_traffic_rules consistent.
* Updating package.xml files to format 3.
* Contributors: Fabian Poggenhans, Joshua Whitley

1.0.1 (2020-03-24)
------------------
* Make sure lanelet2 buildtool_export_depends on mrt_cmake_modules
* Add changelogs
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* Started traffic rules refactoring, bugfixing in traffic rules and extend unittests
* Fix compiler errors with gcc 5
* Initial commit
* Contributors: Fabian Poggenhans, Maximilian Naumann
