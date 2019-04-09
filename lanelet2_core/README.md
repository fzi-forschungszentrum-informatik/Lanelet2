# Lanelet2 Core

For an overview on the Lanelet2 architecture, see [here](doc/Architecture.md)

This package contains the core library of Lanelet2:
- The [basic primitives](doc/LaneletPrimitives.md), including LaneletMap
- [Geometry functions](doc/GeometryPrimer.md)

For usage examples, please refer to the [lanelet2_examples](../lanelet2_examples/README.md) package.

## Debugging

Debugging Lanelet objects can be annoying, because they are very deeply nested.
To improve this, Lanelet2 offers helpers for **GDB** and **QtCreator**. Have a look [here](./res).