# Tagging Regulatory Elements

Regulatory Elements are divided into categories. The most common ones are `TrafficLight`, `TrafficSign`, `SpeedLimit` and `RightOfWay`, which are already included in the core library, but there are many more ways to model restrictions on lanelets and areas. More might be added in the future and also users are able to add own regulatory elements by inheriting from the generic `RegulatoryElement` class and registering the new type using the `RegisterRegulatoryElement` class.

This document describes the generic layout of a Regulatory Element and shows how the common Regulatory Elements are structured.

## Tags

Regulatory Elements always have `type=regulatory_element`. If this tag is not present, Lanelet2 will add it when writing to an .osm file.


### Subtype
The `subtype` tag helps Lanelet2 to distinguish between the different regulatory elements. For the basic Regulatory Elements this would be:
* *traffic_sign*
* *traffic_light*
* *speed_limit*
* *right_of_way*
* *all_way_stop*

### Other, Optional Tags
The following tags can be used to add more information to a Regulatory Element (of course you can add you own to enhance your map and implement a new `TrafficRule` object that implements them). The default values for the tag are highlighted.

* *dynamic* (yes/**no**): Indicates that this Regulatory Element might change its meaning based on a condition. Examples would be a road that is closed on weekends. Or a speed limit that is only in action if the road is wet. By default, Lanelet2 cannot handle dynamic Regulatory Elements and will ignore them. Specialized traffic rule classes could be implemented that use background information (such as the current time) to resolve dynamic Regulatory Elements.
* *fallback* (yes/**no**): Indicates that this Regulatory Element has a lower priority than another Regulatory Element. Examples are right of way regulations that become valid if the traffic lights of an intersection are out of order.


## Parameters

The main feature of a Regulatory Element is that it can reference other parts of the map that are important for the traffic restriction that they represent. These parts are called *parameters* of a Regulatory Element. Every parameter is characterized by a role (a string) that explains what he expresses within the Regulatory Element. Multiple parameters can have the same role if they do not contradict.

An example of parameters are the traffic lights that referenced by the *refers* role of a `TrafficLight` Regulatory Element. These are the traffic lights that a vehicle has to pay attention to when driving along a specific lanelet/area that has this Regulatory Elements. Because parameters with the same role cannot contradict, this means all traffic lights must refer to the same driving direction within that intersection.

The most common roles that are used across all regulatory elements are:
* *refers*: The primitive(s) that are the very origin of the restriction. Traffic lights/signs, et cetera. Most Regulatory Elements need a *refers* role.
* *cancels*: The primitive(s) that mark the end of the restriction (if applicable).
* *ref_line*: The line (usually a LineString) from which a restrictions becomes valid. If not used, that usually means that the whole lanelet/area is affected by the restriction. However, there are exceptions, e.g. for traffic lights the stop line is the *end* of the lanelet.
* *cancel_line*: The line (usally a LineString) from which a restriction is no longer in place (if applicable)

## Basic Regulatory Elements

### Traffic Sign

A traffic sign generically expresses a restriction that is expressed by a traffic sign. The *refers* part refers to traffic signs that form the rule. The *cancels* parameter then refers to traffic signs that mark the end of the restriction expressed by the sign (e.g. the end of no-overtaking section). The *ref_line* and *cancel_line* parameters can then be used to define the exact start and end points of the rule. The LineStrings referenced by that must not have an intersection with the referencing lanelet or Area. If they do, the rule is valid from/to this intersection point. If not, the rule is valid for the whole lanelet/area.

### Speed Limit

Speed limits work very similar to traffic signs. If they are put up by a traffic sign, they simply reference this traffic sign. Similar for the *ref_line* and the *cancels* role. The `TrafficRules` class then takes care of interpreting the speed limit from the `subtype` of the referenced traffic sign.

Alternatively, if the speed limit does not originate from a traffic sign, a `sign_type` tag can be used to define the speed limit. The value should contain the unit, eg "50 km/h". mph or mps or similar units are possible as well. If no unit is given, km/h is assumed.

### Traffic Light

Traffic lights are also similar to traffic signs. Instead of a sign, the light itself is referenced as *refers* parameter. The *cancels* and *cancels_line* role have no meaning for traffic lights. The *ref_line* can reference the respective stop line. If they are not present, the stop line is implicitly at the end of the lanelet or Area.

### Right of Way

By default, intersecting lanelets are treated as a "first come first served" situation, meaning that the vehicle that arrives first at the intersection point has right of way. The `RightOfWay` Regulatory Element changes this. It has three roles:
* *yield*: References the lanelets that have to yield
* *right_of_way*: the lanelets that have the right of way over the yielding ones
* *ref_line*: The lines where vehicles that are crossing a *yield* lanelet have to stop at. If not set, this is the end of the *yield* lanelet.

Only one lanelet of a chain of lanelets that belong to the same lane have to be referenced. Generally this is the last lanelet that can be undoubtedly assigned to one specific intersection arm (i.e. the last lanelet before the intersection begins). All lanelets that are mentioned by the right of way Regulatory Element also have to reference the regulatory element.

### All Way Stop

While in a *Right of Way* regelem, the right of way only depends on the lanelet, the right of way in an [All-Way Stop](https://en.wikipedia.org/wiki/All-way_stop) regelem depends on the order of arrival and the route through the intersection. Therefore, all lanelets are potentially yield lanelets. All approaching vehicles have to stop before entering the intersection.
The intersection entry is either defined by one stop line for each lanelet or is otherwise determined by the end of each lanelet. To avoid confusion when matching lanelets and stop lines, an *All Way Stop* regelem is only valid if either no lanelet has a stop line or all lanelets have exactly one.
The following roles are used in an all way stop:
* *yield*: References the lanelets that might have to yield
* *ref_line*: The lines where *yield* lanelets have to stop. This either empty or has the same order and size as the number of lanelets in *yield*.
* *refers*: The traffic sign(s) that constitute this rule

All lanelets referenced in this regelem also have to reference this regelem.

## OSM XML Examples

Below are some examples of regulatory element relations as they might look in lanelet2's OSM XML format.

### Traffic Light

```xml
<relation id='1'>
  <tag k='type' v='regulatory_element' />
  <tag k='subtype' v='traffic_light' />
  <!-- stop line -->
  <member type='way' ref='2' role='ref_line' />
  <!-- traffic light -->
  <member type='way' ref='3' role='refers' />
</relation>
```

### Speed Limit

```xml
<relation id='1'>
  <tag k='subtype' v='speed_limit' />
  <tag k='type' v='regulatory_element' />
  <!-- Traffic sign(s) that constitute this rule -->
  <member type='way' ref='2' role='refers' />
</relation>
```

### All Way Stop

Below is an example of a 4-way stop intersection.
```xml
<relation id='1'>
  <tag k='type' v='regulatory_element' />
  <tag k='subtype' v='all_way_stop' />
  <!-- Lanelets participating in the 4-way stop -->
  <member type='relation' ref='2' role='yield' />
  <member type='relation' ref='3' role='yield' />
  <member type='relation' ref='4' role='yield' />
  <member type='relation' ref='5' role='yield' />
  <!-- Traffic signs that constitute this rule -->
  <member type='way' ref='6' role='refers' />
  <member type='way' ref='7' role='refers' />
  <member type='way' ref='8' role='refers' />
  <member type='way' ref='9' role='refers' />
  <!-- stop lines for the lanelets (same order as lanelets) -->
  <member type='way' ref='10' role='ref_line' />
  <member type='way' ref='11' role='ref_line' />
  <member type='way' ref='12' role='ref_line' />
  <member type='way' ref='13' role='ref_line' />
</relation>
```

