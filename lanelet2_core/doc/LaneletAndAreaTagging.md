# Tagging Lanelets and Areas

This page explains the general mapping scheme for Lanelets. The tags are used by Lanelet2 to infer information on the traffic rules here. All Lanelets and Areas must be "self sustained" which means that all traffic restrictions that apply to a lanelet must be determined just by looking at the Lanelet. The following important informations have to be determinable:
- The driving direction (only for Lanelets, by default assumed to be one-directional)
- The road users that the lanelet is for (by default for vehicles only)
- The speed limit (by default the maximum speed for urban regions)

No tags are mandatory, but if you want your map to behave reasonably, you should make sure that at least these things can be correctly inferred.


## Subtype and Location

The **subtype** tag determines what the actual type of the lanelet is. It is used to determine the participants that are allowed to use the lanelet as well as partly to determine the speed limit in the absence of *SpeedLimit* regulatory elements. The location tag is used to distinguish between urban and nonurban regions which can (depending on the country) affect the speed limit.

The *subtype* tag is not set, *lane* is assumed. If *location* is not set, urban is assumed. Note that speed limits can be *mandatory* or *non-mandatory*. Non-mandatory speed limits just recommendations. The determined speed limit is the minimum of the actual speed limit and the average speed of a participant on this road type. If it was based on average speed, it is assumed to be non-mandatory.

Note that the values in the following table are just the values inferred by default. The actual interpretation depends on the chosen `TrafficRule`object in `lanelet2_traffic_rules`.

| **subtype** | **location** | **description**                  | **Inferred Participants** | **Speed limit**  | 
|-------------|--------------|----------------------------------|------------------|------------------|
| **road**    | **urban**    | A part of a road in urban region | All vehicles and bikes | City speed limit |
| **road**    | **nonurban** | A part of a road in nonurban region | All vehicles and bikes | Nonurban speed limit |
| **highway** | **urban**    | A part of a highway in urban region | All vehicles  | Urban highway limit |
| **highway** | **nonurban**   | A part of a highway in nonurban region | All vehicles  | Nonurban highway limit |
| **play_street** | **-**    | A part of a play street          | Vehicles, bikes, pedestrians | play street speed limit |
| **emergency_lane** | **-** | Lane for emergency vehicles      | Emergency vehicles | Average emergency vehicle speed |
| **bus_lane** | **urban**       | Lane for buses                   | Bus, Emergency, Taxi | City speed limit |
| **bus_lane** | **nonurban**       | Lane for buses                   | Bus, Emergency, Taxi | Nonurban speed limit |
| **bicycle_lane** | **-**   | A lane that is usable only for bikes | Bikes        | Average bike speed |
| **exit**    | **urban**    | Exit area of a house or garage that crosses the crosswalk   | Vehicles, bikes, pedestrians | Urban speed limit |
| **walkway** | **-**        | A part of a way for pedestrians  | Pedestrians      | Average pedestrian walking speed |
| **shared_walkway** | **-** | A way shared by bikes and pedestrians | Bikes, Pedestrians | Average bike/pedestrian speed |
| **crosswalk** | **-**      | A part of a crosswalk            | Pedestrians      | Average pedestrian walking speed |
| **stairs**  | **-**        | Well ... stairs                  | Pedestrians      | Average pedestrian walking speed |

### Overriding
Since the inferred information above might be incorrect in some situations, it can be overridden by explicitly specifying participants or speed limit. If one of the participants is set like this, the *subtype* and *location* tags are ignored and all other participants are assumed to be disallowed. The allowed participants then have to be set one by one.

The following particiants can be used to define individual special situations. They are prefixed with the type of information they are supposed to override. Values are always *yes* or *no*:
* vehicle (affects all "motorized" participants) *OR*
  * vehicle:car *OR*
    * vehicle:car:electric
    * vehicle:car:combustion
  * vehicle:bus
  * vehicle:truck
  * vehicle:motorcycle
  * vehicle:taxi
  * vehicle:emergency
* pedestrian
* bicycle

To override the participant information, prefix the participants with *participant:*, e.g. *participant:vehicle:car=yes*. This would allow only cars on this lanelet/area.

The normal *vehicle* tag can not be combined with any *vehicle:xxx* tag. If one of the *vehicle:xxx* tags is defined, all other *vehicle:xxx* tags must be set individually.

The speed limit can be overridden separately by using the follwing tags (*speed_limit* must be set, *speed_limit_mandatory* is optional):
* *speed_limit* (any velocity, preferably with unit, e.g. `5 km/h`, no unit is inferred as `km/h`)
* *speed_limit_mandatory* (yes or no, default is yes)

For an even finer overriding behaviour, use *speed_limit:xxx* and *speed_limit_mandatory:xxx*, where xxx is one of the participants from above. If the fine overriding is used and no match can be found for the current participant, *speed_limit* is returned. If it is not set, 0 km/h is returned.

Note that the speed limit can be overriden by `RegulatoryElements`.

The general rule of thumb should be to **avoid the overriding mechanism** where possible. The information can be transported much cleaner by either creating a new `TrafficRule` class that covers road types that we do not cover or by using `RegulatoryElements`.

#### Example
Consider a lanelet with the following tags:

| **Tag** | **value** |
|---------|-----------|
| type    | lanelet   |
| subtype | road      |
| location| urban     |
| participant:vehicle:taxi | yes  |
| participant:vehicle:bus | yes   |
| participant:pedestrian | yes    |
| one_way:pedestrian | no |

Since overriding is used, the subtype/location combination is ignored when determining allowed participants. This means that only the following participants are allowed: Taxi, Bus, Pedestrian. Because the speed limit is not overridden, it could be determined (depending on the country) as 50 km/h (the speed limit for vehicles in cities) for taxis and buses and as non-mandatory 4km/h for pedestrians, since the average walking speed of a pedestrian is assumed to be 4 km/h which is smaller than 50 km/h.

The *one_way* tag - see the section "Direction" below - allows pedestrians in both directions (which is generally a good idea).

## Other, Optional Tags
* road_name (the name of the road)
* road_surface (dirt, asphalt, concrete, ...)
* region: the ISO 3166-2 code for the country in which this lanelet/area is situated. The traffic rules might check this tag to ensure they are working on the correct part of the map.

## Lanelet Specific

All Lanelets have `type=lanelet`. If the tag is not present, it will be added when saving to .osm, because this is a requirement for writing from/to osm files.

Note that the lane change restrictions are not inferred based on the lanelet tags, but on the tags on the borders. See [here](LinestringTagging.md) for more info.

### Direction

By default, Lanelets are one-directional. The direction is determined by the order in which the left and right bound is set. Whether a lanelet is one- or bi-directional is determined by the *one_way* tag. If it is set to *no*, the lanelet is bidirectional. The *one_way* tag can be further specialized to define the direction for different participants. This can be done by appending the type of the participant (as in Overriding), e.g. `one_way:bicycle=no`. This would mean that the lanelet is one-directional for all participants, except bicycles. `one_way` and `one_way:xxxx` can not be used in combination.

Pedestrians use lanelets always bi-directional, unless overridden.

## Area Specific

All areas have `type=multipolygon` (!) for compability reasons with the osm file format. `type=area` is possible too but will be overwritten when writing map data to an .osm file.

### More subtypes

Additionally to the *subtypes* described above, other tags can be used to define the environment of a road:
* parking
* freespace (road area without a purpose)
* vegetation
* keepout
* building
* traffic_island

These tags can be interpreted by special `TrafficRule` objects to determine the drivable space (e.g. for parking maneuvers or emergency situations).

