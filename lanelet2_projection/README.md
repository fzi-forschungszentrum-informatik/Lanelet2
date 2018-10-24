# Lanelet2 Projection Module

This module implements different projection functions as required by `lanelet2_io`.

## Need for projections
As map data is commonly stored in WGS84 (lat/lon/ele) and lanelet2 operates on metrical data, there is a need for precise projection functions between the two. There are very good projection functions available, but they have to be used **with the correct origin** (close to the location of the map).

Maps loaded with a wrong origin can easily be distorted by 40% and more. Angle calculations will be wrong as well.

For more details, read [here](doc/Map_Projections_Coordinate_Systems.md).

## Supported Projections

 - `UTM.h`: WGS84 (for storage in .osm files) <-> UTM (internal processing) (wrapper of https://sourceforge.net/projects/geographiclib/). This projection has the advantage of being very precise, but all points in the map must fit into one UTM Tile. If points exceed a 100km margin, the map can not be loaded.
 - `Mercator.h`: WGS84 (for storage in .osm files) <-> Local  Mercator as in [liblanelet](https://github.com/phbender/liblanelet/blob/master/Commons/mercator.hpp). Approximates the earth 
