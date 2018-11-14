# Lanelet2 IO

IO Module for parsing and writing LaneletMaps. 

It contains a various reader/writer functions for different formats. Which format will be used is determined the extension of the given filename. If a writer/parser is registered for this extension, it will be chosen automatically.

Currently available IO modules are:
- **OSM (.osm)** writes/loads specialized lanelet maps from OpenStreetMap html files. See [maps module](../lanelet2_maps/README.md) for a primer on this.
- **Binary (.bin)** writes/loads the map to/from an internal bin format. Very efficient for writing and reading but not human readable


## Projections
Most IO modules require a projection from WGS84 (lat/lon) to a local metric coordinate system. To make sure the loaded map is correct in itself it is **very important** to choose the correct origin and the correct projector.

The origin should be as close to where the map is as possible.

For an overview on projections, have a look at the [projection module](../lanelet2_projection/README.md).


## Usage
Here is an example of how to read a file from .osm and write it back out as .bin:
```c++
#include <lanelet2_io/io.h>

std::string filename_in = "mymap.osm";
lanelet::Origin origin(49.0, 8.4);
lanelet::LaneletMapPtr laneletMap = lanelet::load(filenameIn, origin);

std::string filename_out = "mymap.bin";
lanelet::write(filenameOut, *laneletMap);
```

