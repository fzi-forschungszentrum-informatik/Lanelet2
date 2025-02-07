from typing import overload
import lanelet2.io
import lanelet2.core

class Projector:
    def forward(self, point: lanelet2.core.GPSPoint) -> lanelet2.core.BasicPoint3d: ...
    def reverse(self, point: lanelet2.core.BasicPoint3d) -> lanelet2.core.GPSPoint: ...
    def origin(self) -> lanelet2.io.Origin: ...


class MercatorProjector(Projector):
    def __init__(self, origin: lanelet2.io.Origin) -> None: ...


class GeocentricProjector(Projector):
    pass


class LocalCartesianProjector(Projector):
    def __init__(self, origin: lanelet2.io.Origin) -> None: ...


class UtmProjector(Projector):
    @overload
    def __init__(self, origin: lanelet2.io.Origin) -> None: ...

    @overload
    def __init__(self, origin: lanelet2.io.Origin, useOffset: bool,
                 throwInPaddingArea: bool) -> None: ...
