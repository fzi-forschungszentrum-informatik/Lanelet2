# Lanelet2 python

Provides python bindings for Lanelet2 and its modules.

## Usage

After building and sourcing you can do something like the follwoing in python(2):
```python
import lanelet2
map = lanelet2.io.load("myfile.osm", lanelet2.io.Origin(49,8.4))

# Modify/Add attribute to all lanelets
for elem in map.laneletLayer:
    if "participant:vehicle" in elem.attributes:
        elem.attributes["participant:vehicle"] = "no"
```

For more usage examples refer to our [example package](../lanelet2_examples/README.md).
