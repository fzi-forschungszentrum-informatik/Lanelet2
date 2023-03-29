# Architecture

This file describes the technical architecture of Lanelet2. For information on the representation of Lanelet2 and its primitives, please read first [here](LaneletPrimitives.md).

# Principles

## Data sharing
In Lanelet2, everything that has an ID is unique across the whole map. Because multiple primitives can reference the same element, it is not possible to duplicate/copy the information of a Lanelet primitive. If that was possible, modifying the information would leave the map in an invalid state, because other elements that reference it would not be notified of the change.

To solve this issue, Lanelet2's primitive do not actually store data. Instead, they hold a *pointer* to the real, uncopyable data object. This means they only provide a *view* on the underlying map data. This means that Lanelet2 primitives can be copied without regret, because all copies still point to the same underlying data object. If the data is modified through one of the primtives, all other copies can observe the change.

This results in some interesting properties. Firstly, primitives can be copied extremely fast, because only the pointer is copied, not the data. Secondly, this means that we can provide different views on the data. One example is that we can give you a 2D view and a 3D view on the data, e.g. a Point2d that returns x and y coordinates but not the z coordinate. You can convert this point back to Point3d without losing information, because in the underlying data, the z-coordinate was always there. Linestrings behave similar. A `LineString3d` returns `Point3d`, a `LineString2d` gives you `Point2d`.

We can also easily *invert* Linestrings and Lanelets with this technique. An inverted Linestring simply returns the underlying data in reversed order. You will not even notice it is inverted, because it still behaves in the same way as a non-inverted one. The effort of creating the inverted Linestring is - you guessed it - just the effort of copying a pointer!

Like this we can make sure that modifying the map is always consistent. All primitives will observe the change. However, there are two exceptions to this, and they are related to caching: The **centerline** of a Lanelet is calculated based on the left and right bound at the time it was first requested. If the points of a left or right bound were modified, the Lanelet does not notice the change and returns the outdated centerline. You have to reset the centerline of the Lanelet yourself. The second issue is within the `LaneletMap` container. It holds some precalculated tree structures to efficiently query closest points or usages of a point. If one of the points is modified, the query will still run on the old tree structure. So the general message is: When you plan to modify the map, know what you are doing!

## Composability
Since Lanelet2's primitives, especially *Lanelets* represent an atomic section of the map, it is often important to compose these atomic parts together to create *compound* objects. These compound primitives behave in the same way as the primitives they are composed of, but internally access the primitive's data. This is also driven by the pointer-based concept introduced above: The compound objects simply hold a list of pointers instead of a single one. As an example, you can compose multiple `Linestring3d` to one `CompoundLineString3d`. It behaves like a single Linestring, gives you its `size()` in points and access to the individual points while still internally accessing the data of the actual Linestrings. You can also compose polygons from Linestrings, `LaneletSet` from Lanelets, and so on.

## Const correctness
Since modifying the map can make cached data invalid, and since modifications affect the whole map, Lanelet2 offers some protection against unwanted modification. This is related to *const correctness*: If an object is passed to a function as `const`, not only the data of the object itself is immutable, but also the data derived from it and all the copies that you make.

E.g., if a function accepts a Linestring as `ConstLinestring3d`, its data is guaranteed to be immutable. If you access a point of the Linestring, you get a `ConstPoint3d`, that allows you to access its data, but not modify it. It is not possible to convert a `ConstPoint3d` back to a `Point3d`. This means, if you call a function that accepts a `const LineString3d` or even a `ConstLineString3d`, it is guaranteed that your map data will not be modified.

## Modularity
We are aware of the fact that roads can be very different in different countries and different places. Some things can be hard to squeeze into the typical map format. Also, the requirements on the map can be very different. To account for this, we tried to make Lanelet2 as flexible as possible by adding customization points where you can plugin your customized solution. Also the modularity of Lanelet2 aims to make it as simple as possible to add new functionality in the future.

Part of the flexibility concept is that the tags that are used on objects can be extended without any limits. This way you can easily add more specific information to your maps that you are missing. New, custom regulatory elements can be added to account for difficult traffic situations. Also Lanelet2 can be extended for different countries and different road participants by adding new `TrafficRules` objects which are used by Lanelet2 to interpret the map data. New parsers and writers for new map formats can be added and registered while still using the same good old `load`/`write` function.

## Geometry calculations
Lanelet2's objects are meant to be directly usable for geometry calculations. They are all registered with boost::geometry, meaning the follwing is easily possible: `double d = boost::geometry::distance(laneletPoint1, laneletPoint2)`. If laneletPoint1/2 is a 2D point, you will get the result in 2D, else in 3D.

However, there are limitations to this that originate from the fact that the ConstCorrectness concept and boost::geometry do not play well with each other, because boost::geometry gets confused by the different point types used when things are used in a const and a non-const context. If you want to know more how to solve this problem and avoid pages and pages of compiler errors from boost's feared template code, read our [Geometry Primer](GeometryPrimer.md) on this.

# Overview and Interaction
If you don't know Lanelet2's basic primitives yet, better read [here](LaneletPrimitives.md) first!

Here, we want to introduce the basic terms and objects that you will be confronted with when using Lanelet2 and how they interact:
* **Primitive** any Lanelet2 primitive and their derivates (`Lanelet`, `ConstLanelet`, `LineString2d`, etc)
* **LaneletMap** a `LaneletMap` is the basic storage container for primitives. It is separated in layers, one for each primitive type and offers different ways to access its data (by a `BoundingBox`, by ID, by nearest point, etc). It does **not** provide routing functionality.
* **TrafficRules** a traffic rules object interprets the map. E.g., it reports if a Lanelet `isPassable`, or if lane changes are possible between two Lanelets. A traffic rule object interprets the map from the perspective of one road participant type. A vehicle `TrafficRule` object will therefore give completely different results on a specific Lanelet than a pedestrian `TrafficRule` object.
* **RoutingCost** these classes are used by the routing graph to determine costs when driving from one Lanelet/area to another one. It could be by travelled distance, by travel time but there are no limits for more advanced routing cost functions. You can also choose the cost of lane changes so that routes with few, preferably long lane changes are preferred.
* **RoutingGraph** a routing graph is built from a `LaneletMap`, `TrafficRules` and `RoutingCost` objects. One routing graph is only for one single participant: The one that the `TrafficRules` belong to. With the routing graph, you can make all kinds of queries to determine where you or someone else can go/drive.
* **Route** a route is something returned by the graph when you query a route from A to B. It contains a structure of all the Lanelets that you can use along the way with the lowest routing cost, including all possible lane changes.
* **LaneletPath** or **Path** in general is a sequence of Lanelets returned by the RoutingGraph that are directly adjacent and have the lowest routing cost to the destination. "Adjacent" means that they can also be connected by a lane change, not only by following the Lanelet in a straight direction.
* **LaneletSequence** a list of directly succeeding Lanelets that can be reached without lane changes. A **LaneletSequence** is the special case of a **LaneletPath** where no lane change is necessary.
* **Projector** projectors are used by the IO module to convert between maps that store date in the WGS84 (lat/lon) format and the local coordinates used by Lanelet2. There are many different projections that all have different properties, so you should choose the one that fits best to you. If in doubt, use UTM.
