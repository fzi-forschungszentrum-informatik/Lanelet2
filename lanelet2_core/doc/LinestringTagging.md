# Tagging Linestrings

This page tells you about the general tags that you should use to annotate your map.

**Note:** All tags are always lowercase.

In general there are three different context in which linestrings are used: As lanelet/area boundary, as a Symbol on the road or as traffic sign/light.

Linestrings are generally characterized by their centerline.


## As Lane Boundary
Tagging linestrings as lane boundary correctly is important, because Lanelet2 uses the type to determine possible lane changes. If Lanelet2 cannot determine the type, Lanelet2 will assume that lane changes are not possible. Here is a list of the most important ones:

| **type** | **subtype** | **description** | **lane change?** |
|----------|-------------|-----------------|------------------|
|line_thin | solid    | a solid lane marking | no       |
|line_thin | solid_solid | a double solid lane marking | no       |
|line_thin | dashed      | a dashed lane marking | yes        |
|line_thin | dashed_solid | a marking that is dashed on the left and solid on the right | left->right: yes |
|line_thin | solid_dashed | the other way around | right->left: yes |
|line_thick | same as above for thin |     |                  |
|curbstone | high        | a curbstone that is to high for a vehicle to drive over | no |
|curbstone | low         | curb that is low and can be driven for a vehicle | no |
|virtual   | -           | a non-physical lane boundary, intended mainly for intersections | no |
|road_border | -         | the end of the road. | no          |

Notice the difference between curbstone and road_border: A road border indicates the end of potentially passable area (e.g. start of vegetation) while a curbstone means that there is passable space (e.g. sidewalk) on the other side of the curb.

There are much more tags (all imply lane changing is not possible):
* guard_rail
* wall
* fence
* zebra_marking (lanelets for pedestrians)
* pedestrian_marking (lines on pedestrian crossings)
* bike_marking (i.e. dashed line for bikes)
* keepout
* virtual
* jersey_barrier
* gate
* door
* rail (trains...)

Feel free to "invent" new types if these do not satisfy your needs, but be aware that future parts of Lanelet2 might make use of the tags mentioned here (e.g. to precisely predict participants) and will fail to interpret custom types.

### Lane Change
If you are unhappy with the types derived by default, you can overwrite it with these tags:
* lane_change=yes (lane change in both directions possible) *OR*
* lane_change:left=<yes/no>, lane_change:right=<yes/no>. Both pairs have to be set if this is used.

### More, Optional Tags
* **width** with of the line (in m). The linestring then represents the *centerline* of the object.
* **height** height of line (in m). The linestring then represents the *lower outline/lowest edge* of the object.
* **temporary** to indicate lines from construction sites (yes/no).
* **color** of the lane marking. White is the default.

### Individual Dashes
Since dashed lines are represented by a full connecting line and not as individual dashes, the information about the position of the dashes is lost. To keep the information, individual points that mark the start and end points of the linestring can be tagged. The `type` tag is used for that. `type=begin` marks the begin of the dash (in the orientation of the linestring), `type=end` marks the end. Other values can be used for other types:
* pole (to mark the position of individual poles on guardrails or fences)
* dot (to mark dots that form line markings)

## Symbols
There can be many different symbols so the selection here is of course incomplete. Symbols can be relevant for some traffic rules

**Arrows** have type="arrow" and a subtype for the direction:
subtype:
* left
* right
* straight
* straight_left
* straight_right
* left_right

Other types:
* stop_line
* zig-zag
* lift_gate
* bump
* 30/50/70 (for speed limits on the road)

The symbol can be represented either by its outline or by its centerline. If it is represented by the centerline, the linestring must only contain two points to avoid confusion with the outline.

## Traffic Signs
Traffic signs all have `type=traffic_sign`. Additionally, they have a "subtype"-tag that contains the actual type of the traffic sign. This subtype is encoded as ISO 3166 region code + traffic sign number (e.g. `subtype=de206` [for a German stop sign](https://de.wikipedia.org/wiki/Bildtafel_der_Verkehrszeichen_in_der_Bundesrepublik_Deutschland_seit_2017#Gefahrzeichen_nach_Anlage_1_(zu_%C2%A7_40_Absatz_6_und_7_StVO)) or `subtype=usR1-1` [for a US stop sign](https://en.wikipedia.org/wiki/Road_signs_in_the_United_States#R1_Series:_Stop_and_Yield)).

Traffic signs can be represented either by their outline (as polygon) or by a linestring where the first point is the left edge of the sign and the last point is the right edge of the sign. Optionally the height tag can be used to encode the size of the sign in z (not the height above ground, use the z-coordinate for that). The linestring marks the *lower edge* of the sign.

## Traffic Lights
Traffic signs have `type=traffic_light`. Similar to traffic signs, they can be represented either by a polygon or a linestring.

The `subtype` tag can be used to add further information on the traffic light:
* red_yellow_green
* red_yellow
* red
* etc.
