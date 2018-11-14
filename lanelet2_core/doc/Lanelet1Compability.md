# Migration from Lanelet1 (liblanelet)

Lanelet1, formerly called liblanelet can be found here: https://github.com/fzi-forschungszentrum-informatik/liblanelet

The maps following the Lanelet2 specification can be loaded using the old implementation. They will load all lanelets without differing between lanelet types. Areas are ignored. Loading new maps in old lanelet1 implementation is a good approach for a seamless migration.

(Early implementations were not able to load maps with node ids larger than 32bit integer range (used in [mapping_example.osm](../lanelet2_maps/res/mapping_example.osm)), but this was fixed in v1.2.)

Note, that extended features introduced in v1.1 and v1.2 like rudimentary traffic light handling, parking places and event regions are not fully compatible with Lanelet2 map data. In the Lanelet2 map specification these were redesigned from scratch.

If you create new maps and you are still using lanelet1 it is a good idea to use the [validator](../lanelet2_validation) of Lanelet2 to (roughly) validate that it is compatible with Lanelet2.


## New projection methods

Lanelet1 had only one projection method: spherical mercator with a local reference point. This is implemented in Lanelet2 as basic method in [Projection.h](../lanelet2_io/include/lanelet2_io/Projection.h) as `SphericalMercatorProjector`. Use this projection if you come from lanelet1 and want to have the same behavior.

Lanelet2 also supports more precise projections. Consider switching to one of these as soon as your application allows it. See package [lanelet2_projection](../lanelet2_projection) for more details.


## Main differences a lanelet1 user should know

Map format specification:
* Traffic light handling, parking places and event regions have been redesigned/replaced.
* Borders of lanelets now actually have a meaning and are used to infer lane change maneuvers.
* Lanelet2 distinguishes between map representation and interpretation. Depending on the road user, the map is interpreted differently.
* Lanelets can only have one single left and one single right bound (this is required for the lane change feature).
* A new `Area` type has been introduced to handle places of undirected traffic (Parking lots, pedestrian walks, etc).

Implementation:
* The implementation has been completely redesigned without respect to backwards compatibility. Expect to have lots of code changes when migrating. *But they are worth it!*
* Lanelet2 converts geographic coordinates to a local metric coordinate system when loading the map. This results in higher performance when working on the map data and allows more complex projection methods.
* Maps can be safely modified, copied and saved.
* LineStrip has been renamed to LineString.
* The old LaneletMap has been divided into smaller software parts to make it more customizable and maintainable. Parts of it can be found in the RoutingGraph (holds the topology) and the new LaneletMap (only holds the primitives).
* Primitives (such as points, linestrings and lanelets) have an actual identity and are shared across all things that use them. Modifying the point of one linestring modifies all other references to this point in other linestrings.
* Lanelet2 primitives support 3D calculations and can be inverted or converted to 2d/3d without copying data.
* Lanelet2 directly supports geometry calculations.
* Lanelet2 is thoroughly tested and does not segfault when trying to copy a LaneletMap :wink:.

