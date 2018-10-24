# Lanelet2 Maps

This repo contains tools for creating and modifying maps. 

## Editing Lanelet2 maps

Lanelet2 maps are best edited using the OpenStreetMap-Editor [JOSM](https://josm.openstreetmap.de/). The josm-Folder of this package contiains styles and presets for simple set-up.

This repository contains two stylesheets. One for visualizing the physical layer of the map (the markings and borders) and one for visualizing the lanelet/area layer.

To use them in JOSM:
- Unzip the images in `style_images.zip` (containing images for traffic signs)
- Under Preferences->Map Settings -> Map Paint styles -> add (+) select `lanelets.mapcss` and `lines.mapcss` to add the lanelt2 styles to JOSM.
- Under Preferences->Map Settings -> Tagging Presets -> add (+) select `laneletpresets.xml` to add the lanelt2 tagging style to JOSM

After that, you can access the presets via Presets->lanelet2. For faster access, add them to the toolbar in Preferences->Toolbar customization.
