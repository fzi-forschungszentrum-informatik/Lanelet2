#include <pugixml.hpp>
#include <sstream>
#include "Exceptions.h"
#include "io_handlers/Factory.h"
#include "io_handlers/OsmFile.h"
#include "io_handlers/OsmHandler.h"

using namespace std::string_literals;

namespace lanelet {
namespace io_handlers {

using Errors = std::vector<std::string>;

namespace {
// register with factories
RegisterWriter<OsmWriter> regWriter;

class ToFileWriter {
 public:
  static std::unique_ptr<osm::File> writeMap(const LaneletMap& laneletMap, const Projector& projector,
                                             ErrorMessages& errors) {
    ToFileWriter writer;

    writer.writeNodes(laneletMap, projector);
    writer.writeWays(laneletMap);

    // we have to wait until lanelets/areas are written
    UnparsedLaneletParameters unparsedLaneletParameters;
    UnparsedAreaParameters unparsedAreaParameters;
    std::tie(unparsedLaneletParameters, unparsedAreaParameters) =
        writer.appendRegulatoryElements(laneletMap.regulatoryElementLayer);
    writer.appendLanelets(laneletMap.laneletLayer);
    writer.appendAreas(laneletMap.areaLayer);
    writer.resolveUnparsedMembers(unparsedLaneletParameters);
    writer.resolveUnparsedMembers(unparsedAreaParameters);

    writer.buildErrorMessage(errors);
    return std::move(writer.file_);
  }

 private:
  ToFileWriter() : file_{std::make_unique<osm::File>()} {}

  // writers for every primitive
  void writeNodes(const LaneletMap& map, const Projector& projector) {
    auto& osmNodes = file_->nodes;
    for (const auto& point : map.pointLayer) {
      try {
        const GPSPoint gpsPoint = projector.reverse(point);
        osmNodes.emplace(point.id(), osm::Node(point.id(), getAttributes(point.attributes()), gpsPoint));
      } catch (ReverseProjectionError& e) {
        writeError(point.id(), e.what());
      }
    }
  }

  void writeWays(const LaneletMap& map) {
    auto& osmWays = file_->ways;
    for (const auto& lineString : map.lineStringLayer) {
      if (lineString.inverted()) {
        writeOsmWay(lineString.invert(), osmWays);
      } else {
        writeOsmWay(lineString, osmWays);
      }
    }
    for (const auto& polygon : map.polygonLayer) {
      writeOsmWay(polygon, osmWays);
    }
  }

  void appendLanelets(const LaneletLayer& laneletLayer) {
    for (const auto& lanelet : laneletLayer) {
      const auto id = lanelet.id();
      auto attributes = getAttributes(lanelet.attributes());
      attributes.emplace(AttributeNamesString::Type, AttributeValueString::Lanelet);
      auto& insertedRelation = file_->relations.emplace(id, osm::Relation(id, attributes)).first->second;
      auto& members = insertedRelation.members;
      tryInsertMembers(members, RoleNameString::Left, lanelet.leftBound().id(), file_->ways, id);
      tryInsertMembers(members, RoleNameString::Right, lanelet.rightBound().id(), file_->ways, id);
      if (lanelet.hasCustomCenterline()) {
        tryInsertMembers(members, RoleNameString::Centerline, lanelet.centerline().id(), file_->ways, id);
      }
      for (const auto& regElem : lanelet.regulatoryElements()) {
        tryInsertMembers(members, RoleNameString::RegulatoryElement, regElem->id(), file_->relations, id);
      }
    }
  }

  void appendAreas(const AreaLayer& areaLayer) {
    for (const auto& area : areaLayer) {
      const auto id = area.id();
      auto attributes = getAttributes(area.attributes());
      attributes.emplace(AttributeNamesString::Type, AttributeValueString::Multipolygon);
      auto& insertedRelation = file_->relations.emplace(id, osm::Relation(id, attributes)).first->second;
      auto outerIds = area.outerBoundPolygon().ids();
      auto innerIds = utils::concatenate(area.innerBoundPolygons(), [](const auto& elem) { return elem.ids(); });
      auto& members = insertedRelation.members;
      for (const auto& outerId : outerIds) {
        tryInsertMembers(members, RoleNameString::Outer, outerId, file_->ways, id);
      }
      for (const auto& innerId : innerIds) {
        tryInsertMembers(members, RoleNameString::Inner, innerId, file_->ways, id);
      }
      for (const auto& regElem : area.regulatoryElements()) {
        tryInsertMembers(members, RoleNameString::RegulatoryElement, regElem->id(), file_->relations, id);
      }
    }
  }

  template <typename PrimitiveT>
  void resolveUnparsedMembers(const std::vector<std::tuple<std::string, PrimitiveT, osm::Relation*>>& unparsedMembers) {
    auto& relations = file_->relations;
    for (const auto& param : unparsedMembers) {
      const auto id = std::get<1>(param).id();
      try {
        std::get<2>(param)->members.emplace(std::get<0>(param), &relations.at(id));
      } catch (std::out_of_range&) {
        writeError(id, "Lanelet/Area has a regulatory element with id "s + std::to_string(std::get<2>(param)->id) +
                           " that is not in the map!");
      }
    }
  }

  // helper functions for writing primitives
  void writeError(Id id, const std::string& what) {
    errors_.push_back("Error writing primitive "s + std::to_string(id) + ": " + what);
  }

  osm::Attributes getAttributes(const AttributeMap& attributes) {
    osm::Attributes osmAttributes;
    for (const auto& attr : attributes) {
      osmAttributes.emplace(attr.first, attr.second.value());
    }
    return osmAttributes;
  }

  template <typename PrimT>
  void writeOsmWay(const PrimT& mapWay, osm::Ways& osmWays) {
    const auto id = mapWay.id();
    const auto wayAttributes = getAttributes(mapWay.attributes());
    try {
      const auto wayNodes =
          utils::transform(mapWay, [& nodes = file_->nodes](const auto& elem) { return &nodes.at(elem.id()); });
      osmWays.emplace(id, osm::Way(id, std::move(wayAttributes), std::move(wayNodes)));
    } catch (NoSuchPrimitiveError& e) {
      writeError(id, "Way has points that are not point layer: "s + e.what());
    } catch (std::out_of_range&) {
      writeError(id, "Way has a point that is not in the map!");
    }
  }

  using UnparsedLaneletParameter = std::tuple<std::string, ConstLanelet, osm::Relation*>;
  using UnparsedAreaParameter = std::tuple<std::string, ConstArea, osm::Relation*>;
  using UnparsedLaneletParameters = std::vector<UnparsedLaneletParameter>;
  using UnparsedAreaParameters = std::vector<UnparsedAreaParameter>;

  class WriteRegulatoryElementVisitor : public RuleParameterVisitor {
   public:
    WriteRegulatoryElementVisitor(osm::File& file, ToFileWriter& writer) : file{file}, writer{writer} {}

    void operator()(const ConstPoint3d& p) override {
      try {
        currRelation->members.emplace(role, &file.nodes.at(p.id()));
      } catch (std::out_of_range&) {
        writer.writeError(
            id, "Regulatory element has parameters that are not in the point layer: "s + std::to_string(p.id()));
      }
    }
    void operator()(const ConstLineString3d& l) override {
      try {
        currRelation->members.emplace(role, &file.ways.at(l.id()));
      } catch (std::out_of_range&) {
        writer.writeError(
            id, "Regulatory element has parameters that are not in the line string layer: "s + std::to_string(l.id()));
      }
    }
    void operator()(const ConstPolygon3d& p) override {
      try {
        currRelation->members.emplace(role, &file.ways.at(p.id()));
      } catch (std::out_of_range&) {
        writer.writeError(
            id, "Regulatory element has parameters that are not in the polygon layer: "s + std::to_string(p.id()));
      }
    }
    void operator()(const ConstWeakLanelet& wll) override {
      // try to lock
      if (wll.expired()) {
        writer.writeError(id, "Found an expired lanelet parameter with role " + role);
        return;
      }
      // lanelets converted to the map yet
      unparsedLaneletParameters.emplace_back(std::make_tuple(role, wll.lock(), currRelation));
    }
    void operator()(const ConstWeakArea& war) override {
      // try to lock
      if (war.expired()) {
        writer.writeError(id, "Found an expired lanelet parameter with role " + role);
        return;
      }
      // lanelets converted to the map yet
      unparsedAreaParameters.emplace_back(std::make_tuple(role, war.lock(), currRelation));
    }
    Id id{0};
    osm::Relation* currRelation{nullptr};
    osm::File& file;
    ToFileWriter& writer;
    UnparsedLaneletParameters unparsedLaneletParameters;
    UnparsedAreaParameters unparsedAreaParameters;
  };

  std::pair<UnparsedLaneletParameters, UnparsedAreaParameters> appendRegulatoryElements(
      const RegulatoryElementLayer& regElemLayer) {
    WriteRegulatoryElementVisitor visitor(*file_, *this);
    for (const auto& regElem : regElemLayer) {
      const auto id = regElem->id();
      auto attributes = getAttributes(regElem->attributes());
      attributes.emplace(AttributeNamesString::Type, AttributeValueString::RegulatoryElement);
      auto& insertedRelation = file_->relations.emplace(id, osm::Relation(id, attributes)).first->second;
      visitor.id = id;
      visitor.currRelation = &insertedRelation;
      regElem->applyVisitor(visitor);
    }
    return std::make_pair(visitor.unparsedLaneletParameters, visitor.unparsedAreaParameters);
  }

  template <typename PrimitiveMap>
  void tryInsertMembers(osm::Roles& insertMembers, const char* insertRole, Id insertId, PrimitiveMap& primitiveMap,
                        Id relationId) {
    try {
      insertMembers.emplace(insertRole, &primitiveMap.at(insertId));
    } catch (std::out_of_range&) {
      writeError(relationId, "Relation has a member with id "s + std::to_string(insertId) + " that is not in the map!");
    }
  }
  void buildErrorMessage(ErrorMessages& errs) const {
    errs.clear();
    if (!errors_.empty()) {
      errs.reserve(errors_.size() + 1);
      errs.emplace_back("Errors ocurred while parsing Lanelet Map:");
      for (const auto& err : errors_) {
        errs.emplace_back("\t- " + err);
      }
    }
  }
  Errors errors_;
  std::unique_ptr<osm::File> file_;
};

}  // namespace

void OsmWriter::write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& errors) const {
  auto file = toOsmFile(laneletMap, errors);
  auto doc = osm::write(*file);
  auto res = doc->save_file(filename.c_str());
  if (!res) {
    throw ParseError("Pugixml failed to write the map (unable to create file?)");
  }
}

std::unique_ptr<osm::File> OsmWriter::toOsmFile(const LaneletMap& laneletMap, ErrorMessages& errors) const {
  return ToFileWriter::writeMap(laneletMap, projector(), errors);
}
}  // namespace io_handlers
}  // namespace lanelet
