# Lanelet2 Validation

Validation package for lanelet2. Runs checks on common mapping errors and reports them.

## Usage

Building this package will create the program `lanelet2_validate`. To test a map, simply run `lanelet2_validate <mymap>`, or better `lanelet2_validate <mymap> --lat <lat> --lon <lon>`, where lat/lon is the origin of your map. The tool will output errors and warnings that were found in your map.

For advanced usage, try `lanelet2_validate --help`.

## Adding Your Own Validators

Before implementing your check, choose a suitable validator. There are three different types of validators:
* *Map validators* validate the primitives in the map and search for obvious mapping issues (tags, positioning, etc.)
* *Traffic rule validators* look for primitives that cannot be interpreted with the traffic rules chose by the user
* *Routing graph validators* check for issues in the routing graph, such as isolated nodes.

Validators should check for one single thing only. Better implement too much validators than too few.

To implement a validator, inherit from one of the base classes [BasicValidator.cpp](include/BasicValidator.cpp),
and implement its operator() and the `name()` function. The name should be in the format "<check_type>.<check_name>".
Then in a `*.cpp` file, register your validator with one of the registry functions like this:
```c++
namespace {
// or RegisterTrafficRuleValidator or RegisterRoutingGraphValidator...
validation::RegisterMapValidator<MyCheckClass> register;
} // namespace
```
