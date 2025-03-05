from typing import Dict, List, Optional, Tuple, overload

import lanelet2.core
import lanelet2.projection


class Origin:
    """
    Represents a geographic origin point.

    Attributes:
        position: The GPS position of the origin.
    """
    @overload
    def __init__(self, lat: float = 0.0, lon: float = 0.0,
                 alt: float = 0.0) -> None: ...

    @overload
    def __init__(self, gpsPoint: lanelet2.core.GPSPoint) -> None: ...

    position: lanelet2.core.GPSPoint


Configuration = Dict[str, str]

@overload
def load(filename: str, projector: lanelet2.projection.Projector = lanelet2.projection.Projector()) -> lanelet2.core.LaneletMap:
    """
    Loads a map from a file using a projector.

    Args:
        filename: The path to the file.
        projector: The projector to use for loading the map.

    Returns:
        The loaded map.
    """
    ...


@overload
def load(filename: str, origin: Origin) -> lanelet2.core.LaneletMap:
    """
    Loads a map from a file using an origin.

    Args:
        filename: The path to the file.
        origin: The origin to use for loading the map.

    Returns:
        The loaded map.
    """
    ...


def loadRobust(filename: str, projector: lanelet2.projection.Projector = lanelet2.projection.Projector()) -> Tuple[lanelet2.core.LaneletMap, List[str]]:
    """
    Loads a map robustly.

    Parser errors are returned as the second member of the tuple.
    If there are errors, the map will be incomplete somewhere.

    Args:
        filename: The path to the file.

    Returns:
        tuple: A tuple containing the loaded map and a list of errors.
    """
    ...


@overload
def write(filename: str, map: lanelet2.core.LaneletMap, projector: lanelet2.projection.Projector, params: Optional[Configuration] = None) -> None:
    """
    Writes the map to a file.

    The extension determines which format will be used (usually .osm).

    Args:
        filename: The path to the file.
        map: The map to write.
        projector: The projector to use for writing the map.
        params: Additional parameters for writing the map.
    """
    ...


@overload
def write(filename: str, map: lanelet2.core.LaneletMap, origin: Origin, params: Optional[Configuration] = None) -> None:
    """
    Writes the map to a file.

    The extension determines which format will be used (usually .osm).

    Args:
        filename: The path to the file.
        map: The map to write.
        origin: The origin to use for writing the map.
        params: Additional parameters for writing the map.
    """
    ...


def writeRobust(filename: str, map: lanelet2.core.LaneletMap, projector: lanelet2.projection.Projector, params: Optional[Configuration] = None) -> List[str]:
    """
    Writes a map robustly and returns writer errors. 

    If there are errors, the map will be incomplete somewhere.

    Args:
        filename: The path to the file.
        map: The map to write.
        projector: The projector to use for writing the map.
        params: Additional parameters for writing the map.

    Returns:
        A list of errors encountered during writing.
    """
    ...
