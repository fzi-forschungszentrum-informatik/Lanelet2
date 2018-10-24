# Lanelet2 python

Provides python bindings for lanelet2 and its modules.

## Usage

After building and sourcing just do the following in pyhthon:
```python
import lanelet2
map = lanelet2.io.load("myfile.osm")

# Modify/Add attribute to all lanelets
for elem in map.laneletLayer:
	if "vehicle" in elem.attributes:
		elem.attributes["vehicle"] = "no"
```
