#include <pugixml.hpp>
#include <sstream>

#include "lanelet2_io/Exceptions.h"
#include "lanelet2_io/io_handlers/Factory.h"
#include "lanelet2_io/io_handlers/OsmFile.h"
#include "lanelet2_io/io_handlers/OsmHandler.h"

using namespace std::string_literals;

namespace lanelet {
namespace io_handlers {

using Errors = std::vector<std::string>;
namespace {
// register with factories
RegisterWriter<OsmWriter> regWriter;

struct UnresolvedRole {
  Id relationId{};
  Id referencedRoleId{};
  osm::Primitive** location{};
};

void removeAndFixPlaceholders(osm::Primitive** toRemove, osm::Roles& fromRoles,
                              std::vector<UnresolvedRole>& placeholders) {
  // find other placeholders that we have to fix
  auto remIt = std::find_if(fromRoles.begin(), fromRoles.end(), [&](auto& role) { return &role.second == toRemove; });
  std::vector<std::pair<size_t, osm::Primitive**>> placeholderLocations;
  for (auto it = fromRoles.begin(); it != fromRoles.end(); ++it) {
    if (it->second == nullptr && remIt != it) {
      placeholderLocations.emplace_back(std::distance(fromRoles.begin(), it), &it->second);
    }
  }
  fromRoles.erase(remIt);
  if (placeholderLocations.empty()) {
    return;  // nothing to update
  }
  // get the new locations
  std::map<osm::Primitive**, osm::Primitive**> newLocations;
  for (auto& loc : placeholderLocations) {
    newLocations.emplace(loc.second, &std::next(fromRoles.begin(), long(loc.first))->second);
  }
  // adapt existing locations
  for (auto& placeholder : placeholders) {
    auto it = newLocations.find(placeholder.location);
    if (it != newLocations.end()) {
      placeholder.location = it->second;
    }
  }
}

class ToFileWriter {
 public:
  static std::unique_ptr<osm::File> writeMap(const LaneletMap& laneletMap, const Projector& projector,
                                             ErrorMessages& errors) {
    ToFileWriter writer;

    writer.writeNodes(laneletMap, projector);
    writer.writeWays(laneletMap);

    // we have to wait until lanelets/areas are written
    auto unparsedLaneletAndAreaParameters = writer.appendRegulatoryElements(laneletMap.regulatoryElementLayer);
    writer.appendLanelets(laneletMap.laneletLayer);
    writer.appendAreas(laneletMap.areaLayer);
    writer.resolveUnparsedMembers(unparsedLaneletAndAreaParameters);

    writer.buildErrorMessage(errors);
    return std::move(writer.file_);
  }

 private:
  ToFileWriter() = default;

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

  void resolveUnparsedMembers(std::vector<UnresolvedRole>& unparsedMembers) {
    auto& relations = file_->relations;
    for (const auto& param : unparsedMembers) {
      const auto id = param.relationId;
      try {
        assert(*param.location == nullptr);
        *param.location = &relations.at(param.referencedRoleId);
      } catch (std::out_of_range&) {
        writeError(id, "Regulatory element has a lanelet/area "s + std::to_string(param.referencedRoleId) +
                           " that is not in the map!");
        // ugly part: clean up the empty dummy at  "location"
        removeAndFixPlaceholders(param.location, relations.at(param.relationId).members, unparsedMembers);
      }
    }
  }

  // helper functions for writing primitives
  void writeError(Id id, const std::string& what) {
    errors_.push_back("Error writing primitive "s + std::to_string(id) + ": " + what);
  }

  static osm::Attributes getAttributes(const AttributeMap& attributes) {
    osm::Attributes osmAttributes;
    for (const auto& attr : attributes) {
      osmAttributes.emplace(attr.first, attr.second.value());
    }
    return osmAttributes;
  }

  template <typename PrimT>
  void writeOsmWay(const PrimT& mapWay, osm::Ways& osmWays) {
    const auto id = mapWay.id();
    auto wayAttributes = getAttributes(mapWay.attributes());
    if (std::is_same<PrimT, ConstPolygon3d>::value) {
      wayAttributes.emplace(AttributeNamesString::Area, "true");
    }
    try {
      const auto wayNodes =
          utils::transform(mapWay, [&nodes = file_->nodes](const auto& elem) { return &nodes.at(elem.id()); });
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
        currRelation->members.emplace_back(role, &file.nodes.at(p.id()));
      } catch (std::out_of_range&) {
        writer.writeError(
            id, "Regulatory element has parameters that are not in the point layer: "s + std::to_string(p.id()));
      }
    }
    void operator()(const ConstLineString3d& l) override {
      try {
        currRelation->members.emplace_back(role, &file.ways.at(l.id()));
      } catch (std::out_of_range&) {
        writer.writeError(
            id, "Regulatory element has parameters that are not in the line string layer: "s + std::to_string(l.id()));
      }
    }
    void operator()(const ConstPolygon3d& p) override {
      try {
        currRelation->members.emplace_back(role, &file.ways.at(p.id()));
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
      // lanelets are not yet in the osm file
      currRelation->members.emplace_back(role, nullptr);
      unparsedLaneletAndAreaParameters.emplace_back(
          UnresolvedRole{currRelation->id, wll.lock().id(), &currRelation->members.back().second});
    }
    void operator()(const ConstWeakArea& war) override {
      // try to lock
      if (war.expired()) {
        writer.writeError(id, "Found an expired lanelet parameter with role " + role);
        return;
      }
      // areas are not yet in the osm file
      currRelation->members.emplace_back(role, nullptr);
      unparsedLaneletAndAreaParameters.emplace_back(
          UnresolvedRole{currRelation->id, war.lock().id(), &currRelation->members.back().second});
    }
    Id id{0};
    osm::Relation* currRelation{nullptr};
    osm::File& file;
    ToFileWriter& writer;
    std::vector<UnresolvedRole> unparsedLaneletAndAreaParameters;
  };

  std::vector<UnresolvedRole> appendRegulatoryElements(const RegulatoryElementLayer& regElemLayer) {
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
    return visitor.unparsedLaneletAndAreaParameters;
  }

  template <typename PrimitiveMap>
  void tryInsertMembers(osm::Roles& insertMembers, const char* insertRole, Id insertId, PrimitiveMap& primitiveMap,
                        Id relationId) {
    try {
      insertMembers.emplace_back(insertRole, &primitiveMap.at(insertId));
    } catch (std::out_of_range&) {
      writeError(relationId, "Relation has a member with id "s + std::to_string(insertId) + " that is not in the map!");
    }
  }
  void buildErrorMessage(ErrorMessages& errs) const {
    errs.clear();
    if (!errors_.empty()) {
      errs.reserve(errors_.size() + 1);
      errs.emplace_back("Errors ocurred while writing Lanelet Map:");
      for (const auto& err : errors_) {
        errs.emplace_back("\t- " + err);
      }
    }
  }
  Errors errors_;
  std::unique_ptr<osm::File> file_{std::make_unique<osm::File>()};
};

void testAndPrintLocaleWarning(ErrorMessages& errors) {
  auto* decimalPoint = std::localeconv()->decimal_point;
  if (decimalPoint == nullptr || *decimalPoint != '.') {
    std::stringstream ss;
    ss << "Warning: Current decimal point of the C locale is set to \""
       << (decimalPoint == nullptr ? ' ' : *decimalPoint) << "\". This will lead to invalid osm output!\n";
    errors.emplace_back(ss.str());
    std::cerr << errors.back();
  }
}
}  // namespace

void OsmWriter::write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& errors) const {
  testAndPrintLocaleWarning(errors);
  auto file = toOsmFile(laneletMap, errors);
  auto doc = osm::write(*file);
  auto res = doc->save_file(filename.c_str(), "  ");
  if (!res) {
    throw ParseError("Pugixml failed to write the map (unable to create file?)");
  }
}

std::unique_ptr<osm::File> OsmWriter::toOsmFile(const LaneletMap& laneletMap, ErrorMessages& errors) const {
  return ToFileWriter::writeMap(laneletMap, projector(), errors);
}
}  // namespace io_handlers
}  // namespace lanelet
