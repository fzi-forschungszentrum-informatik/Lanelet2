# Lanelet2 Maps

This repo contains tools for creating and modifying maps. 

## Editing Lanelet2 maps

Lanelet2 maps are best edited using the OpenStreetMap-Editor [JOSM](https://josm.openstreetmap.de/). The josm-Folder of this package contains styles and presets for simple set-up.

This repository contains two stylesheets. One for visualizing the physical layer of the map (the markings and borders) and one for visualizing the lanelet/area layer.

To use them in JOSM:
- Unzip the images in `style_images.zip` (containing images for traffic signs)
- Under Preferences->Map Settings -> Map Paint styles -> add (+) select `lanelets.mapcss` and `lines.mapcss` to add the lanelt2 styles to JOSM.
- Under Preferences->Map Settings -> Tagging Presets -> add (+) select `laneletpresets.xml` to add the lanelt2 tagging style to JOSM

After that, you can access the presets via Presets->lanelet2. For faster access, add them to the toolbar in Preferences->Toolbar customization.

## Reading and Writing to OSM

Lanelet2 utilizes the [osm xml format](https://wiki.openstreetmap.org/wiki/OSM_XML) as default format to read and write map data. To do this, a simple mapping is done by the library to "transform" lanelet primitives into osm primitives. The Id will be the Id of the primitive (we recommend to use positive Ids, because some OSM editors treat negative Ids as something modifiable and will continue to change them), attributes will be translated to tags of the OSM primitive.

### Points
Points are directly transformed to OSM *Nodes*. The "ele" tag is used to represent the z-coordinate of the point. It denotes the distance to the earth ellipsoid in WGS84.

### LineStrings
LineStrings are transformed to OSM *Ways*.

### Polygons
Polygons are OSM *Ways* as well but are identified by a tag *area=yes*. Start point = end point is not sufficient and also not necessary.

### Lanelets
Lanelets are represented as OSM *relations* with a tag `type=lanelet`. The right bound is a relation with role `right`, the left bound is a role with role `left`, the centerline (if present) is a relation with role `centerline` and all regulatory elements are relations with the role `regulatory_element`.

If there are more relations than the mentioned ones, Lanelet2 will raise an error.

### Areas
Areas are represented as OSM *relations* by making use of the *multipolygon* representation. They have a tag `type=multipolygon`. The outer bound is an ordered list of relations with the role `outer`, the inner bounds are an ordered list of relations with the role `inner`. Lanelet2 parses the inner bounds in this order and starts a new hole whenever the last point of one linestring matches the first one.

### Regulatory Elements
Regulatory elements are also represented as OSM *relations* with `type=regulatory_element` the parameters of a role are directly translated to relations.

All other relations (with no type or a wrong type) will be ignored when parsing .osm data.
