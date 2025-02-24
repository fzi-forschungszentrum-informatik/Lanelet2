from typing import Union
import lanelet2.core


class SpeedLimitInformation:
    """
    Current speed limit as returned by a traffic rule object
    """

    def __init__(self, speedLimitMPS: float, isMandatory: bool = True) -> None:
        """
        Initialize from speed limit [m/s] and bool if speed limit is mandatory.

        Non-mandatory speed limits are usually advisory, but will be used for calculating travel times.
        """
        ...

    @property
    def speedLimit(self) -> float:
        """
        velocity in km/h
        """
        ...

    @speedLimit.setter
    def speedLimit(self, value: float) -> None:
        """
        velocity in km/h
        """
        ...

    @property
    def speedLimitKmH(self) -> float:
        """
        velocity in km/h
        """
        ...

    @speedLimitKmH.setter
    def speedLimitKmH(self, value: float) -> None:
        """
        velocity in km/h
        """
        ...

    @property
    def speedLimitMPS(self) -> float:
        """
        velocity in m/s
        """
        ...

    @speedLimitMPS.setter
    def speedLimitMPS(self, value: float) -> None:
        """
        velocity in m/s
        """
        ...

    isMandatory: bool

    def __str__(self) -> str:
        """
        String representation of SpeedLimitInformation
        """
        ...


class TrafficRules:
    """
    Traffic rules class
    """

    def __init__(self) -> None:
        ...

    def canPass(self, obj: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea], to_obj: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea, None] = None) -> bool:
        """
        Returns whether it is allowed to pass/drive on this lanelet or area, or from one to another.

        If to_obj is None, this function should check if it is allowed to pass on obj.
        If both are not None, it should check if it is allowed to pass from obj to to_obj, assuming both are drivable and connected.
        """
        ...

    def canChangeLane(self, from_lanelet: lanelet2.core.ConstLanelet, to_lanelet: lanelet2.core.ConstLanelet) -> bool:
        """
        Determines if a lane change can be made between two lanelets, assuming both are drivable and adjacent.
        """
        ...

    def speedLimit(self, obj: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea]) -> SpeedLimitInformation:
        """
        Get speed limit of this lanelet or area.
        """
        ...

    def isOneWay(self, lanelet: lanelet2.core.ConstLanelet) -> bool:
        """
        Returns whether a lanelet can be driven in one direction only.
        """
        ...

    def hasDynamicRules(self, lanelet: lanelet2.core.ConstLanelet) -> bool:
        """
        Returns whether dynamic traffic rules apply to this lanelet that cannot be understood by this traffic rules object.
        """
        ...

    @property
    def location(self) -> str:
        """
        Location for which the rules are valid.
        """
        ...

    @property
    def participant(self) -> str:
        """
        Participant for which the rules are valid.
        """
        ...

    def __str__(self) -> str:
        """
        String representation of TrafficRules.
        """
        ...


class Locations:
    """Defines a set of valid location strings for traffic rules."""
    Germany: str


class Participants:
    """Defines a set of valid participant strings for traffic rules."""
    Vehicle: str
    VehicleCar: str
    VehicleCarElectric: str
    VehicleCarCombustion: str
    VehicleBus: str
    VehicleTruck: str
    VehicleMotorcycle: str
    VehicleTaxi: str
    VehicleEmergency: str
    Bicycle: str
    Pedestrian: str
    Train: str


def create(location: str, participant: str) -> TrafficRules:
    """
    Produces a traffic rules object for a given location and participant string.

    See Locations and Participants class for valid strings.
    """
    ...
