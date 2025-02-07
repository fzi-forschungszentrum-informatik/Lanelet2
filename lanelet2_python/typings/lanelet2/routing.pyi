from typing import Any, Iterator, Optional, Union, Callable, List, overload
import lanelet2.core
import lanelet2.traffic_rules


class RoutingCost:
    """
    Object for calculating routing costs between lanelets
    """
    ...

    def getCostSucceeding(self, rules: lanelet2.traffic_rules.TrafficRules, from_: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea], to: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea]) -> float:
        """
        Get the cost of moving from one lanelet/area to another.

        It can be assumed that the lanelets/areas are connected and drivable.
        0 is a legetimate cost, negative values are not allowed.
        Infinite costs are allowed to indicate that a lanelet is not reachable.
        """
        ...

    def getCostLaneChange(self, rules: lanelet2.traffic_rules.TrafficRules, from_: List[lanelet2.core.ConstLanelet], to: List[lanelet2.core.ConstLanelet]) -> float:
        """
        Get the cost of changing lanes from one lane to another.

        It can be assumed that the lanelets are adjacent and drivable.
        The lanelets are ordered from rear to front.
        The first lanelet in the 'from' list will be directly adjacent to the first lanelet in the 'to' list, and so on.
        0 is a legetimate cost, negative values are not allowed.
        Infinite costs are allowed to indicate that a lanelet is not reachable.
        """
        ...


class RoutingCostDistance(RoutingCost):
    """
    Distance based routing cost calculation object
    """

    def __init__(self, laneChangeCost: float, minLaneChangeDistance: float = 0) -> None:
        """
        Initialize RoutingCostDistance with lane change cost and optional minimum lane change distance
        """
        ...


class RoutingCostTravelTime(RoutingCost):
    """
    Travel time based routing cost calculation object
    """

    def __init__(self, laneChangeCost: float, minLaneChangeTime: float = 0) -> None:
        """
        Initialize RoutingCostTravelTime with lane change cost and optional minimum lane change time
        """
        ...


class LaneletPath:
    """
    A set of consecutive lanelets connected in straight direction or by lane changes.
    """

    def __init__(self, lanelets: List[lanelet2.core.ConstLanelet]) -> None:
        """
        Initialize LaneletPath with a list of lanelets.
        """
        ...

    def __iter__(self) -> Iterator[lanelet2.core.ConstLanelet]:
        """
        Iterate over the lanelets in the path.
        """
        ...

    def __len__(self) -> int:
        """
        Get the number of lanelets in the path.
        """
        ...

    def __getitem__(self, index: int) -> lanelet2.core.ConstLanelet:
        """
        Get the lanelet at the specified index.
        """
        ...

    def getRemainingLane(self, llt: lanelet2.core.ConstLanelet) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the sequence of remaining lanelets without a lane change.
        """
        ...

    def __repr__(self) -> str:
        """
        Get the string representation of the LaneletPath.
        """
        ...

    def __eq__(self, other: Any) -> bool:
        """
        Check if two LaneletPaths are equal.
        """
        ...

    def __ne__(self, other: Any) -> bool:
        """
        Check if two LaneletPaths are not equal.
        """
        ...

    def __contains__(self, other: Any) -> bool:
        """
        Check if lanelet is part of the path.
        """


class LaneletVisitInformation:
    """
    Object passed as input for the forEachSuccessor function of the routing graph.
    """
    lanelet: lanelet2.core.ConstLanelet
    predecessor: Optional[lanelet2.core.ConstLanelet]
    length: float
    cost: float
    numLaneChanges: int


class LaneletOrAreaVisitInformation:
    """
    Object passed as input for the forEachSuccessorIncludingAreas function of the routing graph.
    """
    laneletOrArea: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea]
    predecessor: Optional[Union[lanelet2.core.ConstLanelet,
                                lanelet2.core.ConstArea]]
    length: float
    cost: float
    numLaneChanges: int


class PossiblePathsParams:
    """
    Parameters for configuring the behaviour of the possible path algorithms of RoutingGraph.
    """

    def __init__(self, routingCostLimit: Optional[float] = None, elementLimit: Optional[int] = None, routingCostId: int = 0, includeLaneChanges: bool = False, includeShorterPaths: bool = False) -> None:
        """
        Initialize PossiblePathsParams.

        Args:
          routingCostLimit: stop search after given limit
          elementLimit: stop search after given element limit
          routingCostId: Index of routing cost module used for the search
          includeLaneChanges: Whether to consider lane changes
          includeShorterPaths: Whether to also return subpaths, e.g. A, A+B, A+B+C, ...
        """
        ...

    routingCostLimit: Optional[float]
    elementLimit: Optional[int]
    routingCostId: int
    includeLaneChanges: bool
    includeShorterPaths: bool


class LaneletRelation:
    """
    Relation between lanelets.
    """
    lanelet: lanelet2.core.ConstLanelet
    relationType: 'RelationType'

    def __repr__(self) -> str:
        """
        Get the string representation of the LaneletRelation
        """
        ...


class RoutingGraph:
    """
    Main class of the routing module that holds routing information and can be queried.
    """

    def __init__(self, lanelet_map: Union[lanelet2.core.LaneletMap, lanelet2.core.LaneletSubmap], traffic_rules: lanelet2.traffic_rules.TrafficRules, routing_costs: Optional[List['RoutingCost']] = None) -> None:
        """
        Initialize RoutingGraph with a lanelet map, traffic rules, and optional routing costs.

        If no routing costs are given, RoutingCostDistance and RoutingCostTravelTime will be used.
        """
        ...

    def getRoute(self, from_: lanelet2.core.ConstLanelet, to: lanelet2.core.ConstLanelet, routingCostId: int = 0, withLaneChanges: bool = True) -> Optional['Route']:
        """
        Get the route from 'from' to 'to' lanelet.
        """
        ...

    def getRouteVia(self, from_: lanelet2.core.ConstLanelet, via: List[lanelet2.core.ConstLanelet], to: lanelet2.core.ConstLanelet, routingCostId: int = 0, withLaneChanges: bool = True) -> Optional['Route']:
        """
        Get the driving route from 'from' to 'to' lanelet using the 'via' lanelets.
        """
        ...

    def shortestPath(self, from_: lanelet2.core.ConstLanelet, to: lanelet2.core.ConstLanelet, routingCostId: int = 0, withLaneChanges: bool = True) -> Optional[LaneletPath]:
        """
        Get the shortest path between 'start' and 'end' lanelet.
        """
        ...

    def shortestPathWithVia(self, start: lanelet2.core.ConstLanelet, via: List[lanelet2.core.ConstLanelet], end: lanelet2.core.ConstLanelet, routingCostId: int = 0, withLaneChanges: bool = True) -> Optional[LaneletPath]:
        """
        Get the shortest path between 'start' and 'end' lanelet using intermediate points.
        """
        ...

    def routingRelation(self, from_: lanelet2.core.ConstLanelet, to: lanelet2.core.ConstLanelet, includeConflicting: bool = False) -> Optional[LaneletRelation]:
        """
        Get the relation between two lanelets excluding 'conflicting'.
        """
        ...

    def following(self, lanelet: lanelet2.core.ConstLanelet, withLaneChanges: bool = False) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the lanelets that can be reached from this lanelet.
        """
        ...

    def followingRelations(self, lanelet: lanelet2.core.ConstLanelet, withLaneChanges: bool = False) -> List[LaneletRelation]:
        """
        Get the relations to following lanelets.
        """
        ...

    def previous(self, lanelet: lanelet2.core.ConstLanelet, withLaneChanges: bool = False) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the previous lanelets of this lanelet.
        """
        ...

    def previousRelations(self, lanelet: lanelet2.core.ConstLanelet, withLaneChanges: bool = False) -> List[LaneletRelation]:
        """
        Get the relations to preceding lanelets.
        """
        ...

    def besides(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> List[lanelet2.core.ConstLanelet]:
        """
        Get all reachable left and right lanelets, including lanelet, from left to right.
        """
        ...

    def left(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> Optional[lanelet2.core.ConstLanelet]:
        """
        Get the left (routable) lanelet, if it exists.
        """
        ...

    def right(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> Optional[lanelet2.core.ConstLanelet]:
        """
        Get the right (routable) lanelet, if it exists.
        """
        ...

    def adjacentLeft(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> Optional[lanelet2.core.ConstLanelet]:
        """
        Get the left non-routable lanelet.
        """
        ...

    def adjacentRight(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> Optional[lanelet2.core.ConstLanelet]:
        """
        Get the right non-routable lanelet.
        """
        ...

    def lefts(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> List[lanelet2.core.ConstLanelet]:
        """
        Get all left (routable) lanelets recursively.
        """
        ...

    def rights(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> List[lanelet2.core.ConstLanelet]:
        """
        Get all right (routable) lanelets recursively.
        """
        ...

    def adjacentLefts(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> List[lanelet2.core.ConstLanelet]:
        """
        Get all left (non-routable) lanelets recursively.
        """
        ...

    def adjacentRights(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> List[lanelet2.core.ConstLanelet]:
        """
        Get all right (non-routable) lanelets recursively.
        """
        ...

    def leftRelations(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> List[LaneletRelation]:
        """
        Get the relations to left lanelets recursively.
        """
        ...

    def rightRelations(self, lanelet: lanelet2.core.ConstLanelet, routingCostId: int = 0) -> List[LaneletRelation]:
        """
        Get the relations to right lanelets.
        """
        ...

    def conflicting(self, lanelet: lanelet2.core.ConstLanelet) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the conflicting lanelets.

        These are the lanelets whose area overlaps with the given one.
        """
        ...

    def reachableSet(self, lanelet: lanelet2.core.ConstLanelet, maxRoutingCost: float, routingCostId: int = 0, allowLaneChanges: bool = True) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the set of lanelets that can be reached from a given lanelet.
        """
        ...

    def reachableSetTowards(self, lanelet: lanelet2.core.ConstLanelet, maxRoutingCost: float, routingCostId: int = 0, allowLaneChanges: bool = True) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the set of lanelets that can reach a given lanelet.
        """
        ...

    @overload
    def possiblePaths(self, lanelet: lanelet2.core.ConstLanelet, minRoutingCost: float, routingCostId: int = 0, allowLaneChanges: bool = False) -> List[LaneletPath]:
        """
        Get the possible paths from a given start lanelet that are 'minRoutingCost'-long.
        """
        ...

    @overload
    def possiblePaths(self, lanelet: lanelet2.core.ConstLanelet, parameters: PossiblePathsParams) -> List[LaneletPath]:
        """
        Get the possible paths from a given start lanelet as configured in parameters.
        """
        ...

    @overload
    def possiblePathsTowards(self, lanelet: lanelet2.core.ConstLanelet, minRoutingCost: float, routingCostId: int = 0, allowLaneChanges: bool = False) -> List[LaneletPath]:
        """
        Get the possible paths to a given start lanelet that are 'minRoutingCost'-long.
        """
        ...

    @overload
    def possiblePathsTowards(self, lanelet: lanelet2.core.ConstLanelet, parameters: PossiblePathsParams) -> List[LaneletPath]:
        """
        Get the possible paths to a given lanelet as configured in parameters.
        """
        ...

    def possiblePathsMinLen(self, lanelet: lanelet2.core.ConstLanelet, minLanelets: int, allowLaneChanges: bool = False, routingCostId: int = 0) -> List[LaneletPath]:
        """
        Get the possible routes from a given start lanelet that are 'minLanelets'-long
        """
        ...

    def possiblePathsTowardsMinLen(self, lanelet: lanelet2.core.ConstLanelet, minLanelets: int, allowLaneChanges: bool = False, routingCostId: int = 0) -> List[LaneletPath]:
        """
        Get the possible routes from a given start lanelet that are 'minLanelets'-long
        """
        ...

    def forEachSuccessor(self, lanelet: lanelet2.core.ConstLanelet, func: Callable[[LaneletVisitInformation], bool], allowLaneChanges: bool = True, routingCostId: int = 0) -> None:
        """
        Call a function on each successor of lanelet with increasing cost.

        The function must receive a LaneletVisitInformation object as input
        and must return a bool whether followers of the current lanelet should be
        visited as well. The function can raise an exception if an early exit is desired.
        """
        ...

    def forEachSuccessorIncludingAreas(self, lanelet: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea], func: Callable[[LaneletOrAreaVisitInformation], bool], allowLaneChanges: bool = True, routingCostId: int = 0) -> None:
        """
        Similar to forEachSuccessor but also includes areas into the search.
        """
        ...

    def forEachPredecessor(self, lanelet: lanelet2.core.ConstLanelet, func: Callable[[LaneletVisitInformation], bool], allowLaneChanges: bool = True, routingCostId: int = 0) -> None:
        """
        Similar to forEachSuccessor but instead goes backwards along the routing graph.
        """
        ...

    def forEachPredecessorIncludingAreas(self, lanelet: Union[lanelet2.core.ConstLanelet, lanelet2.core.ConstArea], func: Callable[[LaneletOrAreaVisitInformation], bool], allowLaneChanges: bool = True, routingCostId: int = 0) -> None:
        """
        Call a function on each predecessor of lanelet.

        The function must receive a LaneletVisitInformation object as input
        and must return a bool whether followers of the current lanelet should be 
        visited as well. The function can throw an exception if an early exit is desired.
        """
        ...

    def exportGraphML(self, path: str) -> None:
        """
        Export the internal graph to graphML (xml-based) file format.
        """
        ...

    def exportGraphViz(self, path: str) -> None:
        """
        Export the internal graph to graphViz (DOT) file format.
        """
        ...

    def getDebugLaneletMap(self, routingCostId: int = 0, include_adjacent: bool = False, includeConflicting: bool = False) -> lanelet2.core.LaneletMap:
        """
        Get the abstract lanelet map holding the information of the routing graph.
        """
        ...

    def passableLaneletSubmap(self) -> lanelet2.core.LaneletMap:
        """
        Get the LaneletMap that includes all passable lanelets.
        """
        ...

    def checkValidity(self, throw_on_error: bool = True) -> None:
        """
        Perform some basic sanity checks.
        """
        ...


class Route:
    """
    The famous route object that marks a route from A to B, including all lanelets that can be used.
    """

    def shortestPath(self) -> LaneletPath:
        """
        Returns the shortest path along this route.
        """
        ...

    def fullLane(self, lanelet: lanelet2.core.ConstLanelet) -> lanelet2.core.LaneletSequence:
        """
        Returns the complete lane a Lanelet belongs to.

        Empty if lanelet is not on route.
        """
        ...

    def remainingLane(self, lanelet: lanelet2.core.ConstLanelet) -> lanelet2.core.LaneletSequence:
        """
        Returns the remaining lane a Lanelet belongs to.

        Empty if lanelet is not on route.
        """
        ...

    def remainingShortestPath(self, lanelet: lanelet2.core.ConstLanelet) -> LaneletPath:
        """
        Returns all lanelets on the shortest path that follow the input lanelet
        """
        ...

    def length2d(self) -> float:
        """
        Get the 2d length of the shortest path.
        """
        ...

    def numLanes(self) -> int:
        """
        Get the number of individual lanes.
        """
        ...

    def laneletSubmap(self) -> lanelet2.core.LaneletMap:
        """
        Get the laneletSubmap with all lanelets that are part of the route.
        """
        ...

    def getDebugLaneletMap(self) -> lanelet2.core.LaneletMap:
        """
        Get the laneletMap that represents the Lanelets of the Route and their relationship.

        This map will contain linestrings as edges and points as vertices with attributes describing the relationship.
        """
        ...

    def size(self) -> int:
        """
        Get the number of lanelets.
        """
        ...

    def followingRelations(self, lanelet: lanelet2.core.ConstLanelet) -> List[LaneletRelation]:
        """
        Provides the following lanelets within the Route.
        """
        ...

    def previousRelations(self, lanelet: lanelet2.core.ConstLanelet) -> List[LaneletRelation]:
        """
        Provides the previous lanelets within the Route.
        """
        ...

    def leftRelation(self, lanelet: lanelet2.core.ConstLanelet) -> Optional[LaneletRelation]:
        """
        Provides the lanelet left of a given lanelet within the Route.
        """
        ...

    def leftRelations(self, lanelet: lanelet2.core.ConstLanelet) -> List[LaneletRelation]:
        """
        Provides all lanelets left of a given lanelet within the Route.
        """
        ...

    def rightRelation(self, lanelet: lanelet2.core.ConstLanelet) -> Optional[LaneletRelation]:
        """
        Provides the lanelet right of a given lanelet within the Route.
        """
        ...

    def rightRelations(self, lanelet: lanelet2.core.ConstLanelet) -> List[LaneletRelation]:
        """
        Provides all lanelets right of a given lanelet within the Route.
        """
        ...

    def conflictingInRoute(self, lanelet: lanelet2.core.ConstLanelet) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the conflicting lanelets of a lanelet within the route.
        """
        ...

    def conflictingInMap(self, lanelet: lanelet2.core.ConstLanelet) -> List[lanelet2.core.ConstLanelet]:
        """
        Get the conflicting lanelets of a lanelet within all passable lanelets in the laneletMap.
        """
        ...

    def allConflictingInMap(self) -> List[lanelet2.core.ConstLanelet]:
        """
        Get all lanelets in the map that conflict with any lanelet in the route.
        """
        ...

    def forEachSuccessor(self, lanelet: lanelet2.core.ConstLanelet, func: Callable[[LaneletVisitInformation], bool]) -> None:
        """
        Call a function on each successor of lanelet with increasing cost.

        The function must receive a LaneletVisitInformation object as input and
        must return a bool whether followers of the current lanelet should be visited as well.
        The function can raise an exception if an early exit is desired.
        """
        ...

    def forEachPredecessor(self, lanelet: lanelet2.core.ConstLanelet, func: Callable[[LaneletVisitInformation], bool]) -> None:
        """
        Similar to forEachSuccessor but instead goes backwards along the routing graph.
        """
        ...

    def __contains__(self, lanelet: lanelet2.core.ConstLanelet) -> bool:
        """
        Check if a specific lanelet is part of the route.
        """
        ...

    def checkValidity(self) -> None:
        """
        Perform sanity check on the route.
        """
        ...


class RelationType:
    """
    Enum for relation types between lanelets.
    """
    Successor: int
    Left: int
    Right: int
    Conflicting: int
    AdjacentLeft: int
    AdjacentRight: int
