from typing import List, Tuple, Union, overload, Sequence
import lanelet2.core

import lanelet2.core


@overload
def to2D(obj: lanelet2.core.Point2d) -> lanelet2.core.Point2d: ...
@overload
def to2D(obj: lanelet2.core.Point3d) -> lanelet2.core.Point2d: ...
@overload
def to2D(obj: lanelet2.core.BasicPoint3d) -> lanelet2.core.BasicPoint2d: ...
@overload
def to2D(obj: lanelet2.core.BasicPoint2d) -> lanelet2.core.BasicPoint2d: ...
@overload
def to2D(obj: lanelet2.core.ConstPoint2d) -> lanelet2.core.ConstPoint2d: ...
@overload
def to2D(obj: lanelet2.core.ConstPoint3d) -> lanelet2.core.ConstPoint2d: ...
@overload
def to2D(obj: lanelet2.core.LineString2d) -> lanelet2.core.LineString2d: ...
@overload
def to2D(obj: lanelet2.core.ConstLineString2d) -> lanelet2.core.ConstLineString2d: ...
@overload
def to2D(obj: lanelet2.core.LineString3d) -> lanelet2.core.LineString2d: ...
@overload
def to2D(obj: lanelet2.core.ConstLineString3d) -> lanelet2.core.ConstLineString2d: ...
@overload
def to2D(obj: lanelet2.core.Polygon2d) -> lanelet2.core.Polygon2d: ...
@overload
def to2D(obj: lanelet2.core.ConstPolygon2d) -> lanelet2.core.ConstPolygon2d: ...
@overload
def to2D(obj: lanelet2.core.Polygon3d) -> lanelet2.core.Polygon2d: ...
@overload
def to2D(obj: lanelet2.core.ConstPolygon3d) -> lanelet2.core.ConstPolygon2d: ...
@overload
def to2D(obj: lanelet2.core.CompoundLineString2d) -> lanelet2.core.CompoundLineString2d: ...
@overload
def to2D(obj: lanelet2.core.CompoundLineString3d) -> lanelet2.core.CompoundLineString2d: ...


@overload
def to3D(obj: lanelet2.core.Point2d) -> lanelet2.core.Point3d: ...
@overload
def to3D(obj: lanelet2.core.Point3d) -> lanelet2.core.Point3d: ...
@overload
def to3D(obj: lanelet2.core.BasicPoint3d) -> lanelet2.core.BasicPoint3d: ...
@overload
def to3D(obj: lanelet2.core.BasicPoint2d) -> lanelet2.core.BasicPoint3d: ...
@overload
def to3D(obj: lanelet2.core.ConstPoint2d) -> lanelet2.core.ConstPoint3d: ...
@overload
def to3D(obj: lanelet2.core.ConstPoint3d) -> lanelet2.core.ConstPoint3d: ...
@overload
def to3D(obj: lanelet2.core.LineString2d) -> lanelet2.core.LineString3d: ...
@overload
def to3D(obj: lanelet2.core.ConstLineString2d) -> lanelet2.core.ConstLineString3d: ...
@overload
def to3D(obj: lanelet2.core.LineString3d) -> lanelet2.core.LineString3d: ...
@overload
def to3D(obj: lanelet2.core.ConstLineString3d) -> lanelet2.core.ConstLineString3d: ...
@overload
def to3D(obj: lanelet2.core.Polygon2d) -> lanelet2.core.Polygon3d: ...
@overload
def to3D(obj: lanelet2.core.ConstPolygon2d) -> lanelet2.core.ConstPolygon3d: ...
@overload
def to3D(obj: lanelet2.core.Polygon3d) -> lanelet2.core.Polygon3d: ...
@overload
def to3D(obj: lanelet2.core.ConstPolygon3d) -> lanelet2.core.ConstPolygon3d: ...
@overload
def to3D(obj: lanelet2.core.CompoundLineString2d) -> lanelet2.core.CompoundLineString3d: ...
@overload
def to3D(obj: lanelet2.core.CompoundLineString3d) -> lanelet2.core.CompoundLineString3d: ...


@overload
def distance(p1: lanelet2.core.BasicPoint2d,
             p2: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint2d,
             p2: lanelet2.core.ConstPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint2d,
             p2: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.BasicPoint2d,
             p2: lanelet2.core.ConstPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint2d,
             ls: lanelet2.core.ConstHybridLineString2d) -> float: ...


@overload
def distance(ls: lanelet2.core.ConstHybridLineString2d,
             p2: lanelet2.core.ConstPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint2d,
             ls: lanelet2.core.CompoundLineString2d) -> float: ...


@overload
def distance(ls: lanelet2.core.CompoundLineString2d,
             p2: lanelet2.core.ConstPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint2d,
             ls: lanelet2.core.ConstLineString2d) -> float: ...


@overload
def distance(ls: lanelet2.core.ConstLineString2d,
             p2: lanelet2.core.ConstPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.BasicPoint2d,
             ls: lanelet2.core.ConstLineString2d) -> float: ...


@overload
def distance(ls: lanelet2.core.ConstLineString2d,
             p2: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def distance(p1: lanelet2.core.BasicPoint2d,
             ls: lanelet2.core.CompoundLineString2d) -> float: ...


@overload
def distance(ls: lanelet2.core.CompoundLineString2d,
             p2: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def distance(ls1: lanelet2.core.ConstLineString2d,
             ls2: lanelet2.core.ConstLineString2d) -> float: ...


@overload
def distance(ls1: lanelet2.core.ConstHybridLineString2d,
             ls2: lanelet2.core.ConstHybridLineString2d) -> float: ...


@overload
def distance(ls1: lanelet2.core.CompoundLineString2d,
             ls2: lanelet2.core.CompoundLineString2d) -> float: ...


@overload
def distance(ls1: lanelet2.core.ConstLineString2d,
             ls2: lanelet2.core.CompoundLineString2d) -> float: ...


@overload
def distance(ls1: lanelet2.core.CompoundLineString2d,
             ls2: lanelet2.core.ConstLineString2d) -> float: ...

# poly2ls


@overload
def distance(p1: lanelet2.core.ConstPolygon2d,
             p2: lanelet2.core.ConstLineString2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstHybridPolygon2d,
             p2: lanelet2.core.ConstHybridLineString2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstLineString2d,
             p2: lanelet2.core.ConstPolygon2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstHybridLineString2d,
             p2: lanelet2.core.ConstHybridPolygon2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPolygon2d,
             p2: lanelet2.core.CompoundLineString2d) -> float: ...


@overload
def distance(p1: lanelet2.core.CompoundLineString2d,
             p2: lanelet2.core.ConstPolygon2d) -> float: ...

# poly2poly


@overload
def distance(p1: lanelet2.core.ConstHybridPolygon2d,
             p2: lanelet2.core.ConstHybridPolygon2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPolygon2d,
             p2: lanelet2.core.ConstPolygon2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstHybridPolygon2d,
             p2: lanelet2.core.ConstPolygon2d) -> float: ...


@overload
def distance(p1: lanelet2.core.CompoundPolygon2d,
             p2: lanelet2.core.ConstPolygon2d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPolygon2d,
             p2: lanelet2.core.ConstHybridPolygon2d) -> float: ...

# p2llt


@overload
def distance(llt: lanelet2.core.ConstLanelet,
             p: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def distance(llt1: lanelet2.core.ConstLanelet,
             llt2: lanelet2.core.ConstLanelet) -> float: ...


@overload
def distance(p: lanelet2.core.BasicPoint2d,
             llt: lanelet2.core.ConstLanelet) -> float: ...

# p2area


@overload
def distance(llt: lanelet2.core.ConstArea,
             p: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def distance(p: lanelet2.core.BasicPoint2d,
             llt: lanelet2.core.ConstArea) -> float: ...

# 3d
# p2p


@overload
def distance(p1: lanelet2.core.ConstPoint3d,
             p2: lanelet2.core.ConstPoint3d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint3d,
             p2: lanelet2.core.BasicPoint3d) -> float: ...


@overload
def distance(p1: lanelet2.core.BasicPoint3d,
             p2: lanelet2.core.ConstPoint3d) -> float: ...


@overload
def distance(p1: lanelet2.core.BasicPoint3d,
             p2: lanelet2.core.BasicPoint3d) -> float: ...

# p2ls


@overload
def distance(p1: lanelet2.core.ConstPoint3d,
             ls: lanelet2.core.ConstHybridLineString3d) -> float: ...


@overload
def distance(ls: lanelet2.core.ConstHybridLineString3d,
             p2: lanelet2.core.ConstPoint3d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint3d,
             ls: lanelet2.core.CompoundLineString3d) -> float: ...


@overload
def distance(ls: lanelet2.core.CompoundLineString3d,
             p2: lanelet2.core.ConstPoint3d) -> float: ...


@overload
def distance(p1: lanelet2.core.ConstPoint3d,
             ls: lanelet2.core.ConstLineString3d) -> float: ...


@overload
def distance(ls: lanelet2.core.ConstLineString3d,
             p2: lanelet2.core.ConstPoint3d) -> float: ...

# p2lss


def distanceToLines(p: lanelet2.core.ConstPoint2d,
                    ls: Sequence[lanelet2.core.ConstLineString2d]) -> float: ...

# ls2ls


@overload
def distance(ls1: lanelet2.core.ConstLineString3d,
             ls2: lanelet2.core.ConstLineString3d) -> float: ...


@overload
def distance(ls1: lanelet2.core.ConstHybridLineString3d,
             ls2: lanelet2.core.ConstHybridLineString3d) -> float: ...


@overload
def distance(ls1: lanelet2.core.CompoundLineString3d,
             ls2: lanelet2.core.CompoundLineString3d) -> float: ...


@overload
def distance(ls1: lanelet2.core.ConstLineString3d,
             ls2: lanelet2.core.CompoundLineString3d) -> float: ...


@overload
def distance(ls1: lanelet2.core.CompoundLineString3d,
             ls2: lanelet2.core.ConstLineString3d) -> float: ...

# p2llt


@overload
def distance(llt: lanelet2.core.ConstLanelet,
             p: lanelet2.core.BasicPoint3d) -> float: ...


@overload
def distance(llt1: lanelet2.core.BasicPoint3d,
             llt2: lanelet2.core.ConstLanelet) -> float: ...

# p2area


@overload
def distance(llt: lanelet2.core.ConstArea,
             p: lanelet2.core.BasicPoint3d) -> float: ...


@overload
def distance(p: lanelet2.core.BasicPoint3d,
             llt: lanelet2.core.ConstArea) -> float: ...

# equals 2d


@overload
def equals(p1: lanelet2.core.BasicPoint2d,
           p2: lanelet2.core.BasicPoint2d) -> bool: ...


@overload
def equals(p1: lanelet2.core.ConstPoint2d,
           p2: lanelet2.core.ConstPoint2d) -> bool: ...


@overload
def equals(p1: lanelet2.core.ConstPoint2d,
           p2: lanelet2.core.BasicPoint2d) -> bool: ...


@overload
def equals(p1: lanelet2.core.BasicPoint2d,
           p2: lanelet2.core.ConstPoint2d) -> bool: ...

# equals 3d


@overload
def equals(p1: lanelet2.core.BasicPoint3d,
           p2: lanelet2.core.BasicPoint3d) -> bool: ...


@overload
def equals(p1: lanelet2.core.ConstPoint3d,
           p2: lanelet2.core.ConstPoint3d) -> bool: ...


@overload
def equals(p1: lanelet2.core.ConstPoint3d,
           p2: lanelet2.core.BasicPoint3d) -> bool: ...


@overload
def equals(p1: lanelet2.core.BasicPoint3d,
           p2: lanelet2.core.ConstPoint3d) -> bool: ...

# boundingBox2d


@overload
def boundingBox2d(
    obj: lanelet2.core.ConstPoint2d) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.ConstLineString2d) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.ConstHybridLineString2d) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.ConstPolygon2d) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.ConstHybridPolygon2d) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.ConstLanelet) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.ConstArea) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.RegulatoryElement) -> lanelet2.core.BoundingBox2d: ...


@overload
def boundingBox2d(
    obj: lanelet2.core.CompoundLineString2d) -> lanelet2.core.BoundingBox2d: ...


# boundingBox3d
@overload
def boundingBox3d(
    obj: lanelet2.core.ConstPoint3d) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.ConstLineString3d) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.ConstHybridLineString3d) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.ConstPolygon3d) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.ConstHybridPolygon3d) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.ConstLanelet) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.ConstArea) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.RegulatoryElement) -> lanelet2.core.BoundingBox3d: ...


@overload
def boundingBox3d(
    obj: lanelet2.core.CompoundLineString3d) -> lanelet2.core.BoundingBox3d: ...


@overload
def area(obj: List[lanelet2.core.BasicPoint2d]) -> float: ...
@overload
def area(obj: lanelet2.core.ConstHybridPolygon2d) -> float: ...


@overload
def curvature2d(p1: lanelet2.core.BasicPoint2d, p2: lanelet2.core.BasicPoint2d,
                p3: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def curvature2d(
    p1: lanelet2.core.ConstPoint2d, p2: lanelet2.core.ConstPoint2d, p3: lanelet2.core.ConstPoint2d
) -> float: ...


@overload
def signedCurvature2d(p1: lanelet2.core.BasicPoint2d, p2: lanelet2.core.BasicPoint2d,
                      p3: lanelet2.core.BasicPoint2d) -> float: ...


@overload
def signedCurvature2d(
    p1: lanelet2.core.ConstPoint2d, p2: lanelet2.core.ConstPoint2d, p3: lanelet2.core.ConstPoint2d
) -> float: ...


class ArcCoordinates:
    length: float
    distance: float

    def __init__(self, length: float, distance: float) -> None: ...
    def __eq__(self, value: object) -> bool: ...
    def __repr__(self) -> str: ...


@overload
def toArcCoordinates(
    ls: lanelet2.core.ConstLineString2d, p: lanelet2.core.BasicPoint2d
) -> ArcCoordinates: ...


@overload
def toArcCoordinates(
    ls: lanelet2.core.CompoundLineString2d, p: lanelet2.core.BasicPoint2d
) -> ArcCoordinates: ...


@overload
def fromArcCoordinates(
    ls: lanelet2.core.ConstLineString2d, arc: ArcCoordinates
) -> lanelet2.core.BasicPoint2d: ...


@overload
def fromArcCoordinates(
    ls: lanelet2.core.CompoundLineString2d, arc: ArcCoordinates
) -> lanelet2.core.BasicPoint2d: ...


@overload
def length(ls: lanelet2.core.ConstLineString2d) -> float: ...


@overload
def length(ls: lanelet2.core.ConstHybridLineString3d) -> float: ...


@overload
def length(ls: lanelet2.core.CompoundLineString2d) -> float: ...


@overload
def length(ls: lanelet2.core.ConstPolygon2d) -> float: ...


@overload
def interpolatedPointAtDistance(
    ls: List[lanelet2.core.BasicPoint2d], distance: float
) -> lanelet2.core.BasicPoint2d: ...


@overload
def interpolatedPointAtDistance(
    ls: List[lanelet2.core.BasicPoint3d], distance: float
) -> lanelet2.core.BasicPoint3d: ...


@overload
def interpolatedPointAtDistance(
    ls: lanelet2.core.ConstLineString2d, distance: float
) -> lanelet2.core.BasicPoint2d: ...


@overload
def interpolatedPointAtDistance(
    ls: lanelet2.core.ConstLineString3d, distance: float
) -> lanelet2.core.BasicPoint3d: ...


@overload
def interpolatedPointAtDistance(
    ls: lanelet2.core.CompoundLineString2d, distance: float
) -> lanelet2.core.BasicPoint2d: ...


@overload
def interpolatedPointAtDistance(
    ls: lanelet2.core.CompoundLineString3d, distance: float
) -> lanelet2.core.BasicPoint3d: ...


@overload
def nearestPointAtDistance(
    ls: lanelet2.core.ConstLineString2d, distance: float
) -> lanelet2.core.ConstPoint2d: ...


@overload
def nearestPointAtDistance(
    ls: lanelet2.core.ConstLineString3d, distance: float
) -> lanelet2.core.ConstPoint3d: ...


@overload
def nearestPointAtDistance(
    ls: lanelet2.core.CompoundLineString2d, distance: float
) -> lanelet2.core.ConstPoint2d: ...


@overload
def nearestPointAtDistance(
    ls: lanelet2.core.CompoundLineString3d, distance: float
) -> lanelet2.core.ConstPoint3d: ...


@overload
def project(
    ls: lanelet2.core.ConstLineString2d, p: lanelet2.core.BasicPoint2d
) -> lanelet2.core.BasicPoint2d: ...


@overload
def project(
    ls: lanelet2.core.ConstLineString3d, p: lanelet2.core.BasicPoint3d
) -> lanelet2.core.BasicPoint3d: ...


@overload
def project(
    ls: lanelet2.core.CompoundLineString2d, p: lanelet2.core.BasicPoint2d
) -> lanelet2.core.BasicPoint2d: ...


@overload
def project(
    ls: lanelet2.core.CompoundLineString3d, p: lanelet2.core.BasicPoint3d
) -> lanelet2.core.BasicPoint3d: ...


@overload
def projectedPoint3d(
    ls: lanelet2.core.ConstLineString3d, p: lanelet2.core.ConstLineString3d
) -> Tuple[lanelet2.core.BasicPoint3d, lanelet2.core.BasicPoint3d]:
    """Returns the respective projected points of the closest distance of two linestrings"""
    ...


@overload
def projectedPoint3d(
    ls: lanelet2.core.ConstHybridLineString3d, p: lanelet2.core.ConstHybridLineString3d
) -> Tuple[lanelet2.core.BasicPoint3d, lanelet2.core.BasicPoint3d]:
    """Returns the respective projected points of the closest distance of two linestrings"""
    ...


@overload
def projectedPoint3d(
    ls: lanelet2.core.CompoundLineString3d, p: lanelet2.core.CompoundLineString3d
) -> Tuple[lanelet2.core.BasicPoint3d, lanelet2.core.BasicPoint3d]:
    """Returns the respective projected points of the closest distance of two linestrings"""
    ...

# intersects2d


@overload
def intersects2d(ls1: lanelet2.core.ConstLineString2d,
                 ls2: lanelet2.core.ConstLineString2d) -> bool: ...


@overload
def intersects2d(ls1: lanelet2.core.ConstHybridLineString2d,
                 ls2: lanelet2.core.ConstHybridLineString2d) -> bool: ...


@overload
def intersects2d(ls1: lanelet2.core.CompoundLineString2d,
                 ls2: lanelet2.core.CompoundLineString2d) -> bool: ...


@overload
def intersects2d(ls1: lanelet2.core.ConstPolygon2d,
                 ls2: lanelet2.core.ConstPolygon2d) -> bool: ...


@overload
def intersects2d(ls1: lanelet2.core.ConstHybridPolygon2d,
                 ls2: lanelet2.core.ConstHybridPolygon2d) -> bool: ...


@overload
def intersects2d(bbox1: lanelet2.core.BoundingBox3d,
                 bbox2: lanelet2.core.BoundingBox3d) -> bool: ...


@overload
def intersects2d(llt1: lanelet2.core.ConstLanelet,
                 llt2: lanelet2.core.ConstLanelet) -> bool: ...


@overload
def intersects2d(area1: lanelet2.core.ConstArea,
                 area2: lanelet2.core.ConstArea) -> bool: ...

# intersects3d


@overload
def intersects3d(ls1: lanelet2.core.ConstLineString3d,
                 ls2: lanelet2.core.ConstLineString3d, heightTolerance: float = 3) -> bool: ...


@overload
def intersects3d(bbox1: lanelet2.core.BoundingBox3d,
                 bbox2: lanelet2.core.BoundingBox3d) -> bool: ...


@overload
def intersects3d(ls1: lanelet2.core.ConstHybridLineString3d, ls2: lanelet2.core.ConstHybridLineString3d,
                 heightTolerance: float = 3) -> bool: ...


@overload
def intersects3d(ls1: lanelet2.core.CompoundLineString3d, ls2: lanelet2.core.CompoundLineString3d,
                 heightTolerance: float = 3) -> bool: ...


@overload
def intersects3d(lanelet1: lanelet2.core.ConstLanelet,
                 lanelet2: lanelet2.core.ConstLanelet, heightTolerance: float = 3.0) -> bool: ...


def inside(llt: lanelet2.core.ConstLanelet, p: lanelet2.core.BasicPoint2d) -> bool:
    """Tests whether a point is within a lanelet."""
    ...


def length2d(llt: lanelet2.core.ConstLanelet) -> float:
    """Calculate length of centerline in 2D."""
    ...


def approximatedLength2d(llt: lanelet2.core.ConstLanelet) -> float:
    """Approximates length by sampling points along the left bound in 2D."""
    ...


def length3d(llt: lanelet2.core.ConstLanelet) -> float:
    """Calculate length of centerline in 3D."""
    ...


def distanceToCenterline2d(llt: lanelet2.core.ConstLanelet, p: lanelet2.core.BasicPoint2d) -> float:
    """Calculate the distance from a point to the centerline in 2D."""
    ...


def distanceToCenterline3d(llt: lanelet2.core.ConstLanelet, p: lanelet2.core.BasicPoint3d) -> float:
    """Calculate the distance from a point to the centerline in 3D."""
    ...


def overlaps2d(llt1: lanelet2.core.ConstLanelet, llt2: lanelet2.core.ConstLanelet) -> bool:
    """Returns true if the shared area of two lanelets is greater than 0 in 2D."""
    ...


def overlaps3d(llt1: lanelet2.core.ConstLanelet, llt2: lanelet2.core.ConstLanelet, heightTolerance: float = 3.0) -> bool:
    """Approximates if two lanelets overlap (area > 0) in 3D."""
    ...


def intersectCenterlines2d(llt1: lanelet2.core.ConstLanelet, llt2: lanelet2.core.ConstLanelet) -> List[lanelet2.core.BasicPoint2d]:
    """Returns the intersection points of the centerlines of two lanelets in 2D."""
    ...


def leftOf(llt1: lanelet2.core.ConstLanelet, llt2: lanelet2.core.ConstLanelet) -> bool:
    """Returns true if the first lanelet is directly left of the second."""
    ...


def rightOf(llt1: lanelet2.core.ConstLanelet, llt2: lanelet2.core.ConstLanelet) -> bool:
    """Returns true if the first lanelet is directly right of the second."""
    ...


def follows(llt1: lanelet2.core.ConstLanelet, llt2: lanelet2.core.ConstLanelet) -> bool:
    """Returns true if the first lanelet precedes the second."""
    ...


@overload
def findNearest(layer: lanelet2.core.PointLayer, p: lanelet2.core.BasicPoint2d,
                n: int) -> List[Tuple[float, lanelet2.core.Point3d]]: ...


@overload
def findNearest(layer: lanelet2.core.LineStringLayer, p: lanelet2.core.BasicPoint2d,
                n: int) -> List[Tuple[float, lanelet2.core.LineString3d]]: ...


@overload
def findNearest(layer: lanelet2.core.PolygonLayer, p: lanelet2.core.BasicPoint2d,
                n: int) -> List[Tuple[float, lanelet2.core.Polygon3d]]: ...


@overload
def findNearest(layer: lanelet2.core.LaneletLayer, p: lanelet2.core.BasicPoint2d,
                n: int) -> List[Tuple[float, lanelet2.core.Lanelet]]: ...


@overload
def findNearest(layer: lanelet2.core.AreaLayer, p: lanelet2.core.BasicPoint2d,
                n: int) -> List[Tuple[float, lanelet2.core.Area]]: ...


@overload
def findNearest(layer: lanelet2.core.RegulatoryElementLayer, p: lanelet2.core.BasicPoint2d,
                n: int) -> List[Tuple[float, lanelet2.core.RegulatoryElement]]: ...


#   // find within, point layer
#   wrapFindWithin2d<Point3d, Point2d>();
#   wrapFindWithin2d<Point3d, BasicPoint2d>();
#   wrapFindWithin2d<Point3d, BoundingBox2d>();
#   wrapFindWithin2d<Point3d, Polygon2d>();
#   wrapFindWithin2d<Point3d, BasicPolygon2d>();
#   wrapFindWithin2d<Point3d, LineString2d>();
#   wrapFindWithin2d<Point3d, BasicLineString2d>();
#   wrapFindWithin2d<Point3d, CompoundLineString2d>();
#   wrapFindWithin2d<Point3d, Lanelet>();
#   wrapFindWithin2d<Point3d, Area>();
#   wrapFindWithin3d<Point3d, Point3d>();
#   wrapFindWithin3d<Point3d, BasicPoint3d>();
#   wrapFindWithin3d<Point3d, BoundingBox3d>();
#   wrapFindWithin3d<Point3d, Polygon3d>();
#   wrapFindWithin3d<Point3d, BasicPolygon3d>();
#   wrapFindWithin3d<Point3d, LineString3d>();
#   wrapFindWithin3d<Point3d, BasicLineString3d>();
#   wrapFindWithin3d<Point3d, CompoundLineString3d>();
#   wrapFindWithin3d<Point3d, Lanelet>();
#   wrapFindWithin3d<Point3d, Area>();

_Primitives2d = Union[lanelet2.core.Point2d, lanelet2.core.BasicPoint2d, lanelet2.core.BoundingBox2d,
                      lanelet2.core.Polygon2d, lanelet2.core.LineString2d,
                      lanelet2.core.CompoundLineString2d, lanelet2.core.Lanelet,
                      lanelet2.core.Area]
_Primitives3d_1 = Union[lanelet2.core.Point3d, lanelet2.core.BasicPoint3d, lanelet2.core.BoundingBox3d,
                        lanelet2.core.Polygon3d, lanelet2.core.LineString3d,
                        lanelet2.core.CompoundLineString3d, lanelet2.core.Lanelet,
                        lanelet2.core.Area]
_Primitives3d_2 = Union[lanelet2.core.Point3d, lanelet2.core.BasicPoint3d]


@overload
def findWithin2d(layer: lanelet2.core.PointLayer, geometry: _Primitives2d, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Point3d]]:
    """Find points within a certain distance of a geometry in 2D."""
    ...


@overload
def findWithin3d(layer: lanelet2.core.PointLayer, geometry: _Primitives3d_1, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Point3d]]:
    """Find points within a certain distance of a geometry in 3D."""
    ...


@overload
def findWithin2d(layer: lanelet2.core.LineStringLayer, geometry: _Primitives2d, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.LineString3d]]:
    """Find linestrings within a certain distance of a geometry in 2D."""
    ...


@overload
def findWithin3d(layer: lanelet2.core.LineStringLayer, geometry: _Primitives3d_2, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.LineString3d]]:
    """Find linestrings within a certain distance of a geometry in 3D."""
    ...


@overload
def findWithin2d(layer: lanelet2.core.PolygonLayer, geometry: _Primitives2d, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Polygon3d]]:
    """Find polygons within a certain distance of a geometry in 2D."""
    ...


@overload
def findWithin3d(layer: lanelet2.core.PolygonLayer, geometry: _Primitives3d_2, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Polygon3d]]:
    """Find polygons within a certain distance of a geometry in 3D."""
    ...


@overload
def findWithin2d(layer: lanelet2.core.LaneletLayer, geometry: _Primitives2d, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Lanelet]]:
    """Find lanelets within a certain distance of a geometry in 2D."""
    ...


@overload
def findWithin3d(layer: lanelet2.core.LaneletLayer, geometry: _Primitives3d_2, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Lanelet]]:
    """Find lanelets within a certain distance of a geometry in 3D."""
    ...


@overload
def findWithin2d(layer: lanelet2.core.AreaLayer, geometry: _Primitives2d, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Area]]:
    """Find areas within a certain distance of a geometry in 2D."""
    ...


@overload
def findWithin3d(layer: lanelet2.core.AreaLayer, geometry: _Primitives3d_2, maxDist: float = 0.) -> List[Tuple[float, lanelet2.core.Area]]:
    """Find areas within a certain distance of a geometry in 3D."""
    ...


@overload
def intersection(ls1: List[lanelet2.core.BasicPoint2d], ls2: List[lanelet2.core.BasicPoint2d]) -> List[lanelet2.core.BasicPoint2d]:
    """Returns the intersection points of two linestrings."""
    ...


@overload
def intersection(ls1: lanelet2.core.CompoundLineString2d, ls2: lanelet2.core.ConstLineString2d) -> List[lanelet2.core.BasicPoint2d]:
    """Returns the intersection points of two linestrings."""
    ...


@overload
def intersection(ls1: lanelet2.core.CompoundLineString2d, ls2: lanelet2.core.CompoundLineString2d) -> List[lanelet2.core.BasicPoint2d]:
    """Returns the intersection points of two linestrings."""
    ...


@overload
def intersection(ls1: lanelet2.core.ConstLineString2d, ls2: lanelet2.core.ConstLineString2d) -> List[lanelet2.core.BasicPoint2d]:
    """Returns the intersection points of two linestrings."""
    ...
