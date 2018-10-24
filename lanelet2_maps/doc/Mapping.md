## General

Check:
* Upper case is not allowed for any tag or role


## LineStrings (Ways)
-----------------------------------------------
Identified by area!=yes or area undefined

### Tag "type"

Lanelet borders:
* line_thick (lc depends on subtype)
* line_thin (lc depends on subtype)
* curbstone (lc=no)
* guard_rail (lc=no)
* road_border (road ends at vegetation) (lc=no)
* wall (lc=no)
* fence (lc=no)
* zebra_marking (lc=undefined)
* pedestrian_marking (lines on pedestrian crossings) (lc=undefined)
* bike_marking (i.e. dashed line for bikes) (lc=no)
* keepout (lc=no)
* virtual (lc=no)
* jersey_barrier (lc=no)
* rail (trains...) (lc=undefined)

Other flat lines:
* stop_line (lc=undefined)
* visualization (lc=undefined)
* zig-zag (lc=undefined)
* lift_gate (lc=undefined)
* trajectory (lc=undefined)
* bump (speedbump) (lc=undefined)

Other:
* traffic_light
* traffic_sign
* arrow (road marking)
* symbol (road marking)

lc=derived lane_change tag.
things with undefined lc can not be border of a lanelet.

Check:
* All ways must have a type
* lc=undefined is not part of a lanelet for vehicles.
* Tag must exist for a valid lineString

### Tag "subtype"
For line types or curbstones
Lanelet borders:
* straight (default for line_thick/line_thin) (lc=no)
* dashed (lc=yes)
* dashed_straight (lc depends on direction)
* straight_dashed (lc depends on direction)
* straight_straight (lc=no)
* high (curbstone, default) (lc=no)
* low (curbstone) (lc=no)

Other:
* lowercase ISO 3166 region code + any number (encodes a traffic sign id, e.g. de206 for a german stop sign)
* for arrows: straight, left, right, straight or any combination: left-right, ... (?)
* for symbols on the road: 30, 50, 70
* for traffic lights:
    * red_yellow_green (default)
    * red_green
    * red_yellow
    * red
    * yellow

Check:
* List is not final

## Tag "temporary"
* yes
* no

indicates that an element will be removed in future (e.g. on construction sites)

### Tag "area"
* no

Check:
* asBool must work

### Tag "lane_change"
Explicitly allows or forbids changes to the left along this border. This Information is usually derived from "subtype" tag. The direction is determined by the linestring orientation

* yes
* no

### Tag "lane_change:left"

* yes
* no

### Tag "lane_change:right"
Works similar to lane_change:right

* yes
* no

Check:
* lane_change and lane_change:_something_ can not exist together

### Tag "width"
Explicitly state a line width in m.

Check:
* asDouble must work.


## Polygons
-----------------------------------------------
Identified by area=yes

Rarely used. For connected linestrings use Areas.

### Tag "type"
* visualization (can be used to visualize areas of lanelets)

Check:
* All areas must have a type


## Nodes
-----------------------------------------------

### Tag "type"
* pole
* post
* start (indicates start of a dashed line in direction of the linestring)
* end (indicates end of a dashed line)

Check:
* List is not final (check similar spelling)
* Dashed_thin and dashed_thick always need start and end tags somewhere


### Tag "orientation"
Defines the oriantation of something (east=0) in radians. Range is 0, ..., 2pi
Examples are arrows

Check:
* asDouble should never fail on orientation
* Range should not be exceeded

### Tag "variance"
Defines the covariance of a point (models uncertainty in position of a lanelet) in m^2.

Check:
* asDouble should never fail
* positive number

### Tag "ele"
Defines the absolute height in m. This is assumed to be WGS84 (distance to earth geoid). You can use something else (e.g. do everything relative to ground level), but then don't expect your map to be compatible to other maps.

# Relations
=====================================

## Lanelets
------------------------------------------------

### Tag "type"
* lanelet

Check:
* Must exist

### Tag "subtype"
Used to determine traffic rules.
* play_street
* normal (default)
* main_road (?)
* highway
* emergency_lane
* stairs

### Tag "one_way"
* yes (default)
* no

Check:
* List is final

### Tag "region"
A country or region which traffic rules apply
* de (default?)
* iso codes (ISO 3166-2)

Check:
* List is not final

### Tag "location"
Used to determine traffic rules
* nonurban (traffic rules of cities apply)
* urban (traffic rules for urban areas apply)
* private (traffic rules for non-public areas apply)

Check
* List is not final
* Must exist

### Tag "vehicle"
* yes/no

Defines if a lanelet is drivable for vehicles. The other tags below work in the same way.
Vehicle=yes implies car=yes, truck=yes, bus=yes, taxi=yes. Similar for Vehicle=no.

Check:
* One of these tags (at least) must be defined
* vehicle and vehicle:xxx can not exist together.

### Tag "vehicle:car"
* yes/no (default, unless vehicle=true)

This and the following tags allow a finer definition of the values

### Tag "vehicle:truck"
* yes/no (default, unless vehicle=true)

### Tag "vehicle:bus"
* yes/no (default, unless vehicle=true)

### Tag "vehicle:taxi"
* yes/no (default, unless vehicle=true)

Check:
* List is not final

### Tag "vehicle:motorcycle"
* yes/no (default, unless vehicle=true)

Check:
* List is not final

### Tag "pedestrian"
* yes/no (default)

### Tag "bicycle"
* yes/no (default)

### Tag "train"
* yes/no (default)

Check:
* If train is yes, everything else else should be "no"

### Tag "emergency"
* yes/no(default)

For emergency lanes

Check:
* If emergency is yes, everything else else should be "no" (except for maybe taxi and bus)

### Tag "one_way:XXX"
Can be used to define divergent directions for other traffic participants (e.g. one_way:bicycle=no).
one_way:pedestrian=no by default. all others inherit their state from one_way.

### Role "left"
Must exist, defines left lineString

Check:
* Exacly one "left" must exist
* Left.size() >= 1
* left must be a linestring

### Role "right"
Must exist, see "left"

### Role "centerline"
Optional. If it exists, the referenced linestring is taken as the centerline

### Role "regulatory_element"
Rules that apply for the lanelet

## Areas
------------------------------------------------
Identified by: type=multipolygon (!)

Other tag requirements are shared with lanelet

## Tag "subtype"
* parking
* freespace (road area without a purpose)
* vegetation
* walkway
* keepout
* building
* traffic_island
* exit (e.g. of a house)

Check:
* list is not final
* must exist

## Tag "accessible"
* yes/no

Defines if an area could be accessed by a vehicle (regardless whether it is legal or not)

## Role "outer"
Defines the elements of the area. Order is important.

Check:
* Every last point of a border must be identical to the first point of next border (or the last point. will be inverted then).
* Endpoint must be identical to start point
* All borders must be linestrings
* Must exist

## Role "inner"
Defines inner elements of the area. Order still important.


# Regulatory elements
------------------------------------------------
Identified by: type=regulatory_element

Check:
* The generic regulatory_element without a subtype tag should rarely be used. If it occurs, check for common roles like stop_line, traffic_light, traffic_sign, etc.

### Tag "type"
Yields a generic regulatory element
* regulatory_element

### Tag "dynamic"
* yes/no (default is no)

Indicates that the regulatory element is enabled based on a condition. How the condition is specified depends on the subtype of the regulatory element. Examples are dynamic speed limits from digital speed signs. Traffic lights do not count as dynamic regulatory elements. 

## Traffic light
------------------------------------------------
Identified by: subtype=traffic_light

### Role "refers"
Defines the traffic light to look for

Check:
* must exist
* can occur multiple times
* Must be nodes with type="traffic_light"
* Alle nodes must have the same "subtype" (or none)

### Role "ref_line"
Defines stop position for a red light.
* must exist
* exactly one.
* must be a linestring

## Traffic sign
------------------------------------------------
Identified by: subtype=traffic_sign


### Tag "when"
* fallback
* sundays ...
* always (default)

Defines when a rule becomes active. There can be a large bunch of cases. Fallback means that this rule becomes active only if a another rule  does not apply (e.g. a defect traffic light).

### Tag "sign_type"
Can be used as lazy alternative to role "refers" if no traffic sign is referenced. It has the type of the traffic sign "e.g. de274".

### Role "refers"
Points to the traffic sign(s) that define this rule

Check:
* Must exist 
* can occur multiple times
* must be nodes with type="traffic_sign"
* all nodes must have the same "subtype"

### Role "cancels"
Points to the traffic sign(s) that cancel this rule.
Especially for speed limits (see below): Defines where the rule ends.

### Role "ref_line"
Defines the position from where the sign is valid. If it doesnt exist, it is valid for the whole lanelet.

Check:
* Must not exist or only once.
* Must be a linestring

## Speed limit
------------------------------------------------
Identified by subtype=speed_limit
Has the same roles as traffic sign, but the traffic_sign members must define a speedlimit.

## Right of way
------------------------------------------------
Identified by: subtype=right_of_way

Expresses right of way relations between one or multiple lanelets that have to yield over other lanelets.
Every lanelet that is mentioned in the regulatory element as "yield" or "right of way" must hold this regulatory element.

If two intersecting lanelets have no right of way relation, they have equal right of way, meaning the first vehicle to arrive at the intersection has right of way.

### Role "refers"
*Same rules as for a normal traffic_sign relation apply*

The traffic sign that defines this rule, if it exists

### Role "yield"
The lanelet(s) that have to yield

### Role "right_of_way"
The lanelet(s) that have right of way over the yielding lanelets

Check:
* Must exist
* Must be lanelet

### Role "ref_line"
Defines where to stop

Check:
* must exist

## Bump
----------------------------------------------
Identified by subtype=bump.
Defines a speed bump as a regulatory element.

### Role "ref_line"
The position of the speed bump

Check:
* must be line
* must have subtype "speed_bump"


TODO:
* Lanes that change their behaviour depending on traffic (lane direction, emergency lane turned to normal lane, etc)
