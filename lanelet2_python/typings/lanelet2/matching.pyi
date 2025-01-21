from typing import List, overload
import lanelet2.core

import lanelet2.traffic_rules


class Pose2d:
    """
    2D Isometric Transformation.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """
        Initialize Pose2d with x, y, and yaw.
        """
        ...

    def __str__(self) -> str:
        """
        String representation of Pose2d.
        """
        ...


class Object2d:
    """
    Object with pose, hull and ID.
    """

    def __init__(self, object_id: int = 0, pose: Pose2d = Pose2d(), absoluteHull: List[lanelet2.core.BasicPoint2d] = []) -> None:
        """
        Initialize Object2d with object ID, pose, and absolute hull
        """
        ...

    absoluteHull: List[lanelet2.core.BasicPoint2d]
    pose: Pose2d
    objectId: int


class PositionCovariance2d:
    """
    Position covariance matrix.
    """

    def __init__(self, varX: float = 0.0, varY: float = 0.0, covXY: float = 0.0) -> None:
        """
        Initialize PositionCovariance2d with variance in x, variance in y, and covariance between x and y.
        """
        ...

    def __str__(self) -> str:
        """
        String representation of PositionCovariance2d.
        """
        ...


class ObjectWithCovariance2d(Object2d):
    """
    Object with pose, covariance, hull and ID.
    """

    def __init__(self, object_id: int = 0, pose: Pose2d = Pose2d(), absoluteHull: List[lanelet2.core.BasicPoint2d] = [], positionCovariance: PositionCovariance2d = PositionCovariance2d(), vonMisesKappa: float = 0.0) -> None:
        """
        Initialize ObjectWithCovariance2d with object ID, pose, absolute hull, position covariance, and von Mises kappa.
        """
        ...

    vonMisesKappa: float
    positionCovariance: PositionCovariance2d


class ConstLaneletMatch:
    """
    Results from matching objects to lanelets
    """
    lanelet: lanelet2.core.ConstLanelet
    distance: float


class ConstLaneletMatchProbabilistic(ConstLaneletMatch):
    """
    Results from matching objects to lanelets with probabilistic approach
    """
    mahalanobisDistSq: float


def getDeterministicMatches(laneletMap: lanelet2.core.LaneletMap, obj: Object2d, maxDist: float) -> List[ConstLaneletMatch]:
    """
    Get deterministic matches for the given object within the specified maximum distance.
    """
    ...


def getProbabilisticMatches(laneletMap: lanelet2.core.LaneletMap, obj: ObjectWithCovariance2d, maxDist: float) -> List[ConstLaneletMatchProbabilistic]:
    """
    Get probabilistic matches for the given object within the specified maximum distance
    """
    ...


@overload
def removeNonRuleCompliantMatches(matches: List[ConstLaneletMatch], traffic_rules: lanelet2.traffic_rules.TrafficRules) -> List[ConstLaneletMatch]:
    """
    Remove non-rule compliant matches from the given list of matches.
    """
    ...


@overload
def removeNonRuleCompliantMatches(matches: List[ConstLaneletMatchProbabilistic], traffic_rules: lanelet2.traffic_rules.TrafficRules) -> List[ConstLaneletMatchProbabilistic]:
    """
    Remove non-rule compliant matches from the given list of matches.
    """
    ...
