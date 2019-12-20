#pragma once
#include <boost/units/physical_dimensions/acceleration.hpp>
#include <boost/units/physical_dimensions/velocity.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si/base.hpp>
#include <cstdint>
#include <memory>
#include <vector>

/**
 * @namespace lanelet basic namespace for everything in lanelet2
 */
namespace lanelet {

// PrimitiveData
class PrimitiveData;
using PrimitiveDataPtr = std::shared_ptr<PrimitiveData>;
using PrimitiveDataConstPtr = std::shared_ptr<const PrimitiveData>;
using PrimitiveDataPtrs = std::vector<PrimitiveDataPtr>;
using PrimitiveDataConstPtrs = std::vector<PrimitiveDataConstPtr>;

// PointData
class PointData;
using PointDataPtr = std::shared_ptr<PointData>;
using PointDataConstPtr = std::shared_ptr<const PointData>;
using PointDataPtrs = std::vector<PointDataPtr>;
using PointDataConstPtrs = std::vector<PointDataConstPtr>;

// Point
class Point3d;
class Point2d;
class ConstPoint3d;
class ConstPoint2d;
using Points3d = std::vector<Point3d>;
using ConstPoints3d = std::vector<ConstPoint3d>;
using Points2d = std::vector<Point2d>;
using ConstPoints2d = std::vector<ConstPoint2d>;

// GPSPoint
class GPSPoint;
using GPSPoints = std::vector<GPSPoint>;

// LinestringData
class LineStringData;
using LineStringDataPtr = std::shared_ptr<LineStringData>;
using LineStringDataConstPtr = std::shared_ptr<const LineStringData>;
using LineStringDataPtrs = std::vector<LineStringDataPtr>;
using LineStringDataConstPtrs = std::vector<LineStringDataConstPtr>;

// Linestring
class LineString3d;
class ConstLineString3d;
class LineString2d;
class ConstLineString2d;
class ConstHybridLineString2d;
class ConstHybridLineString3d;
using LineStrings3d = std::vector<LineString3d>;
using ConstLineStrings3d = std::vector<ConstLineString3d>;
using LineStrings2d = std::vector<LineString2d>;
using ConstLineStrings2d = std::vector<ConstLineString2d>;
using ConstHybridLineStrings2d = std::vector<ConstHybridLineString2d>;
using ConstHybridLineStrings3d = std::vector<ConstHybridLineString3d>;

// CompoundLineData
class CompoundLineStringData;
using CompoundLineStringDataPtr = std::shared_ptr<CompoundLineStringData>;
using CompoundLineStringDataConstPtr = std::shared_ptr<const CompoundLineStringData>;
using CompoundLineStringDataPtrs = std::vector<CompoundLineStringDataPtr>;
using CompoundLineStringDataConstPtrs = std::vector<CompoundLineStringDataConstPtr>;

// CompoundLineString
class CompoundLineString3d;
class CompoundLineString2d;
class CompoundHybridLineString2d;
class CompoundHybridLineString3d;
using CompoundLineStrings2d = std::vector<CompoundLineString2d>;
using CompoundLineStrings3d = std::vector<CompoundLineString3d>;
using CompoundHybridLineStrings2d = std::vector<CompoundHybridLineString2d>;
using CompoundHybridLineStrings3d = std::vector<CompoundHybridLineString3d>;

// CompoundPolygon
class CompoundPolygon3d;
class CompoundPolygon2d;
class CompoundHybridPolygon2d;
class CompoundHybridPolygon3d;
using CompoundPolygons3d = std::vector<CompoundPolygon3d>;
using CompoundPolygons2d = std::vector<CompoundPolygon2d>;
using CompoundHybridPolygons2d = std::vector<CompoundHybridPolygon2d>;
using CompoundHybridPolygons3d = std::vector<CompoundHybridPolygon3d>;

// LaneletSequenceData
class LaneletSequenceData;
using LaneletSequenceDataPtr = std::shared_ptr<LaneletSequenceData>;
using LaneletSequenceDataConstPtr = std::shared_ptr<const LaneletSequenceData>;
using LaneletSequenceDataPtrs = std::vector<LaneletSequenceDataPtr>;
using LaneletSequenceDataConstPtrs = std::vector<LaneletSequenceDataConstPtr>;

// LaneletData
class LaneletData;
using LaneletDataPtr = std::shared_ptr<LaneletData>;
using LaneletDataptr = std::weak_ptr<LaneletData>;
using LaneletDataConstPtr = std::shared_ptr<const LaneletData>;
using LaneletDataConstWptr = std::weak_ptr<const LaneletData>;
using LaneletDataPtrs = std::vector<LaneletDataPtr>;
using LaneletDataConstPtrs = std::vector<LaneletDataConstPtr>;
using LaneletDataConstWptrs = std::vector<LaneletDataConstWptr>;

// Lanelet
class Lanelet;
class ConstLanelet;
class WeakLanelet;
class ConstWeakLanelet;
using Lanelets = std::vector<Lanelet>;
using ConstLanelets = std::vector<ConstLanelet>;

// LaneletSequence
class LaneletSequence;
using LaneletSequences = std::vector<LaneletSequence>;

// Area
class Area;
class ConstArea;
class WeakArea;
class ConstWeakArea;
using Areas = std::vector<Area>;
using ConstAreas = std::vector<ConstArea>;

// AreaData
class AreaData;
using AreaDataPtr = std::shared_ptr<AreaData>;
using AreaDataptr = std::weak_ptr<AreaData>;
using AreaDataConstPtr = std::shared_ptr<const AreaData>;
using AreaDataPtrs = std::vector<AreaDataPtr>;
using AreaDataConstPtrs = std::vector<AreaDataConstPtr>;

// LaneletOrArea
class ConstLaneletOrArea;
using ConstLaneletOrAreas = std::vector<ConstLaneletOrArea>;

// Polygon
class BasicPolygon2d;
class BasicPolygon3d;
class Polygon3d;
class ConstPolygon3d;
class Polygon2d;
class ConstPolygon2d;
class ConstHybridPolygon2d;
class ConstHybridPolygon3d;
using Polygons3d = std::vector<Polygon3d>;
using ConstPolygons3d = std::vector<ConstPolygon3d>;
using Polygons2d = std::vector<Polygon2d>;
using ConstPolygons2d = std::vector<ConstPolygon2d>;
using BasicPolygons2d = std::vector<BasicPolygon2d>;
using BasicPolygons3d = std::vector<BasicPolygon3d>;
using ConstHybridPolygons2d = std::vector<ConstHybridPolygon2d>;
using ConstHybridPolygons3d = std::vector<ConstHybridPolygon3d>;

// polygon with holes
class BasicPolygonWithHoles3d;
class BasicPolygonWithHoles2d;
using BasicPolygonsWithHoles3d = std::vector<BasicPolygonWithHoles3d>;
using BasicPolygonsWithHoles2d = std::vector<BasicPolygonWithHoles2d>;

// LaneletMap
class LaneletMap;
using LaneletMapPtr = std::shared_ptr<LaneletMap>;
using LaneletMapUPtr = std::unique_ptr<LaneletMap>;
using LaneletMapConstPtr = std::shared_ptr<const LaneletMap>;
using LaneletMapConstUPtr = std::unique_ptr<const LaneletMap>;
using LaneletMapPtrs = std::vector<LaneletMapPtr>;
using LaneletMapConstPtrs = std::vector<LaneletMapConstPtr>;

// LaneletSubmap
class LaneletSubmap;
using LaneletSubmapPtr = std::shared_ptr<LaneletSubmap>;
using LaneletSubmapUPtr = std::unique_ptr<LaneletSubmap>;
using LaneletSubmapConstPtr = std::shared_ptr<const LaneletSubmap>;
using LaneletSubmapConstUPtr = std::unique_ptr<const LaneletSubmap>;
using LaneletSubmapPtrs = std::vector<LaneletSubmapPtr>;
using LaneletSubmapConstPtrs = std::vector<LaneletSubmapConstPtr>;

// RegulatoryElementData
class RegulatoryElementData;
using RegulatoryElementDataPtr = std::shared_ptr<RegulatoryElementData>;
using RegulatoryElementDataConstPtr = std::shared_ptr<const RegulatoryElementData>;
using RegulatoryElementDataPtrs = std::vector<RegulatoryElementDataPtr>;
using RegulatoryElementDataConstPtrs = std::vector<RegulatoryElementDataConstPtr>;

// RegulatoryElement
class RegulatoryElement;
class GenericRegulatoryElement;
using RegulatoryElementPtr = std::shared_ptr<RegulatoryElement>;
using RegulatoryElementPtrs = std::vector<RegulatoryElementPtr>;
using RegulatoryElementConstPtr = std::shared_ptr<const RegulatoryElement>;
using RegulatoryElementConstPtrs = std::vector<RegulatoryElementConstPtr>;
class RegulatoryElementFactory;

using Id = int64_t;
constexpr Id InvalId = 0;  //!< indicates a primitive that is not part of a map
using Ids = std::vector<Id>;

// Velocity definitions
namespace units {
using MPS = boost::units::unit<boost::units::velocity_dimension, boost::units::si::system>;
using MPS2 = boost::units::unit<boost::units::acceleration_dimension, boost::units::si::system>;
using MPSQuantity = boost::units::quantity<MPS>;
using MPS2Quantity = boost::units::quantity<MPS2>;

}  // namespace units
using Velocity = units::MPSQuantity;
using Acceleration = units::MPS2Quantity;

}  // namespace lanelet
