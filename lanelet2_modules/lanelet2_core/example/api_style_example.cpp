

/* This is more or less pseudo code and illustrates how the usage of the library
 * could look like in
 * the end. Always following the principle "Keep in mind that code is read much
 * more often than
 * written."
 * (https://wiki.qt.io/Coding_Conventions)
 *
 * Also the way PCL separates algorithms into classes with a lot of
 * setters/getters is a good
 * template.
 */

using namespace lanelet;

LaneletMap::Ptr map(new LocalLaneletMap());  // could also be ROSLaneletMap()

Reader::Ptr reader(new OSMReader());  // could also be BinaryReader()

reader->setMap(map);
reader->readFromFile(filename);

Routing : Ptr routing(new LaneletRoutingAlgo1());  // different algos possible
                                                   // with differing complexity

routing->setMap(map);
routing->queryRoute(start_lanelet, goal_lanelet);

routing->queryRoutes(start_lanelet, distance)
