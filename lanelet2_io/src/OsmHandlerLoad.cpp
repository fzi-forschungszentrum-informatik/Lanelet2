#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <boost/geometry/algorithms/is_valid.hpp>
#include <fstream>
#include <iostream>
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
RegisterParser<OsmParser> regParser;
using traits::to2D;
bool isValid(const LineStrings3d& lss) {
  BasicPolygon2d ls(utils::concatenate(lss, [](const auto& elem) { return to2D(elem).basicLineString(); }));
  return boost::geometry::is_valid(ls);
}

void reverse(LineStrings3d& lss) {
  for (auto& ls : lss) {
    ls = ls.invert();
  }
  std::reverse(lss.begin(), lss.end());
}

template <typename PrimT>
PrimT getDummy(Id id) {
  return PrimT(id);
}
template <>
RegulatoryElementPtr getDummy<RegulatoryElementPtr>(Id id) {
  return std::make_shared<GenericRegulatoryElement>(std::make_shared<RegulatoryElementData>(id));
}

Errors buildErrorMessage(const std::string& errorIntro, const Errors& errors) {
  if (errors.empty()) {
    return {};
  }
  Errors message{errorIntro};
  message.reserve(errors.size() + 1);
  for (const auto& error : errors) {
    message.push_back("\t- " + error);
  }
  return message;
}

class FromFileLoader {  // NOLINT
 public:
  static std::unique_ptr<LaneletMap> loadMap(const osm::File& file, const Projector& projector, ErrorMessages& errors) {
    FromFileLoader loader;

    loader.loadNodes(file.nodes, projector);
    loader.loadWays(file.ways);
    auto laneletsWithRelation = loader.loadLanelets(file.relations);
    auto areasWithRelation = loader.loadAreas(file.relations);

    loader.loadRegulatoryElements(file.relations);
    loader.addRegulatoryElements(laneletsWithRelation);
    loader.addRegulatoryElements(areasWithRelation);
    errors = std::move(loader.errors_);
    return std::make_unique<LaneletMap>(loader.lanelets_, loader.areas_, loader.regulatoryElements_, loader.polygons_,
                                        loader.lineStrings_, loader.points_);
  }

 private:
  template <typename PrimT>
  using PrimitiveWithRegulatoryElement = std::pair<PrimT, const osm::Relation*>;

  template <typename PrimT>
  using PrimitivesWithRegulatoryElement = std::vector<PrimitiveWithRegulatoryElement<PrimT>>;

  using AreasWithRegulatoryElements = PrimitivesWithRegulatoryElement<Area>;
  using LaneletsWithRegulatoryElements = PrimitivesWithRegulatoryElement<Lanelet>;

  FromFileLoader() = default;

  void loadNodes(const lanelet::osm::Nodes& nodes, const Projector& projector) {
    for (const auto& nodeElem : nodes) {
      const auto& node = nodeElem.second;
      try {
        points_.emplace(node.id, Point3d(node.id, projector.forward(node.point), getAttributes(node.attributes)));
      } catch (ForwardProjectionError& e) {
        parserError(node.id, e.what());
      }
    }
  }
  void loadWays(const lanelet::osm::Ways& ways) {
    for (const auto& wayElem : ways) {
      const auto& way = wayElem.second;
      // reconstruct points
      Points3d points;
      points = utils::transform(way.nodes,
                                [this, &way](const auto& n) { return this->getOrGetDummy(points_, n->id, way.id); });
      if (points.empty()) {
        parserError(way.id, "Ways must have at least one point!");
        continue;
      }

      const auto id = way.id;
      const auto attributes = getAttributes(way.attributes);

      // determine area or way
      auto isArea = attributes.find(AttributeNamesString::Area);
      if (isArea != attributes.end() && isArea->second.asBool().get_value_or(false)) {
        polygons_.emplace(id, Polygon3d(id, points, attributes));
      } else {
        lineStrings_.emplace(id, LineString3d(id, points, attributes));
      }
    }
  }

  LaneletsWithRegulatoryElements loadLanelets(const lanelet::osm::Relations& relations) {
    // The regulatory elements are not parsed yet. We store lanelets with one
    // for
    // later.
    LaneletsWithRegulatoryElements llWithRegulatoryElement;
    for (const auto& relElem : relations) {
      const auto& llElem = relElem.second;
      if (!isType<AttributeValueString::Lanelet>(llElem)) {
        continue;
      }
      const auto id = llElem.id;
      const auto attributes = getAttributes(llElem.attributes);
      auto left = getLaneletBorder(llElem, RoleNameString::Left);
      auto right = getLaneletBorder(llElem, RoleNameString::Right);

      // correct their orientation
      std::tie(left, right) = geometry::align(left, right);

      // look for optional centerline
      Lanelet lanelet(id, left, right, attributes);
      if (findRole(llElem.members, RoleNameString::Centerline) != llElem.members.end()) {
        auto center = getLaneletBorder(llElem, RoleNameString::Centerline);
        lanelet.setCenterline(center);
      }

      lanelets_.emplace(id, lanelet);

      // check for regulatory elements
      if (findRole(llElem.members, RoleNameString::RegulatoryElement) != llElem.members.end()) {
        llWithRegulatoryElement.push_back(std::make_pair(lanelet, &llElem));
      }
    }
    return llWithRegulatoryElement;
  }

  AreasWithRegulatoryElements loadAreas(const lanelet::osm::Relations& relations) {
    // regElems are not parsed yet. We store areas with one for later.
    AreasWithRegulatoryElements arWithRegulatoryElement;
    for (const auto& relElem : relations) {
      const auto& arElem = relElem.second;
      if (!isType<AttributeValueString::Multipolygon>(arElem)) {
        continue;
      }
      const auto id = arElem.id;
      const auto attributes = getAttributes(arElem.attributes);

      auto outerRing = getOuterRing(arElem);
      if (outerRing.empty()) {
        // getOuter ring repors errors for us
        continue;
      }

      Area area(id, outerRing, getInnerRing(arElem), attributes);
      areas_.emplace(id, area);

      // check for regulatory elements
      if (findRole(arElem.members, RoleNameString::RegulatoryElement) != arElem.members.end()) {
        arWithRegulatoryElement.push_back(std::make_pair(area, &arElem));
      }
    }
    return arWithRegulatoryElement;
  }

  void loadRegulatoryElements(const osm::Relations& relations) {
    for (const auto& relElem : relations) {
      const auto& regElem = relElem.second;
      if (!isType<AttributeValueString::RegulatoryElement>(regElem)) {
        continue;
      }
      const auto id = regElem.id;
      const auto attributes = getAttributes(regElem.attributes);
      const auto type = attributes.find(AttributeName::Subtype);
      if (type == attributes.end()) {
        parserError(id, "Regulatory element has no 'subtype' tag.");
        continue;
      }
      auto rules = getRulesForRegulatoryElement(id, regElem.members);
      auto regelemData = std::make_shared<RegulatoryElementData>(id, rules, attributes);
      auto regelemType = type->second.value();
      try {
        auto regElem = RegulatoryElementFactory::create(regelemType, regelemData);
        regulatoryElements_.emplace(id, regElem);
      } catch (std::exception& e) {
        parserError(id, "Creating a regulatory element of type "s + regelemType + " failed: " + e.what());
      }
    }
  }

  template <typename PrimT>
  void addRegulatoryElements(std::vector<std::pair<PrimT, const osm::Relation*>>& addTos) {
    for (auto& addTo : addTos) {
      osm::forEachMember(addTo.second->members, RoleNameString::RegulatoryElement, [&](const osm::Role& role) {
        auto regElem = getOrGetDummy(regulatoryElements_, role.second->id, addTo.first.id());
        addTo.first.addRegulatoryElement(regElem);
      });
    }
  }

  // helper functions
  template <const char* Type>
  bool isType(const lanelet::osm::Relation& relation) {
    auto attr = relation.attributes.find(AttributeNamesString::Type);
    return attr != relation.attributes.end() && attr->second == Type;
  }

  static lanelet::AttributeMap getAttributes(const lanelet::osm::Attributes& osmAttributes) {
    lanelet::AttributeMap attributes;
    for (const auto& osmAttr : osmAttributes) {
      attributes.insert(std::make_pair(osmAttr.first, lanelet::Attribute(osmAttr.second)));
    }
    return attributes;
  }

  LineString3d getLaneletBorder(const osm::Relation& llElem, const std::string& role) {
    size_t numMembers = 0;
    osm::forEachMember(llElem.members, role, [&](auto& /*role*/) { ++numMembers; });
    if (numMembers != 1) {
      parserError(llElem.id, "Lanelet has not exactly one "s + role + " border!");
      return LineString3d(llElem.id);
    }
    auto member = osm::findRole(llElem.members, role);
    if (member->second->type() != AttributeValueString::Way) {
      parserError(llElem.id, "Lanelet "s + role + " border is not of type way!");
      return LineString3d(llElem.id);
    }
    return getOrGetDummy(lineStrings_, member->second->id, llElem.id);
  }

  LineStrings3d getLinestrings(const osm::Roles& roles, const std::string& roleName, Id refId) {
    LineStrings3d linestrings;
    osm::forEachMember(roles, roleName, [&](auto& member) {
      if (member.second->type() != AttributeValueString::Way) {
        auto msg = roleName + " ring must consist of ways but id " + std::to_string(member.second->id) +
                   " is of type " + member.second->type() + "!";
        msg[0] = std::toupper(msg[0]);
        this->parserError(refId, msg);
        return;
      }
      auto elem = lineStrings_.find(member.second->id);
      if (elem == lineStrings_.end()) {
        this->parserError(refId, "Failed to get id "s + std::to_string(member.second->id) + " from map");
        return;
      }
      linestrings.push_back(elem->second);
    });
    return linestrings;
  }

  LineStrings3d getOuterRing(const osm::Relation& area) {
    auto outerLs = getLinestrings(area.members, RoleNameString::Outer, area.id);
    if (outerLs.empty()) {
      parserError(area.id, "Areas must have at least one outer border!");
      return {};
    }
    auto outerRings = assembleBoundary(outerLs, area.id);
    if (outerRings.size() != 1) {
      parserError(area.id, "Areas must have exactly one outer ring!");
      return {};
    }
    return outerRings.front();
  }

  std::vector<LineStrings3d> getInnerRing(const osm::Relation& area) {
    auto innerLs = getLinestrings(area.members, RoleNameString::Inner, area.id);
    return assembleBoundary(innerLs, area.id);
  }

  RuleParameterMap getRulesForRegulatoryElement(Id currElemId, const osm::Roles& roles) {
    RuleParameterMap rules;
    for (const auto& memberPair : roles) {
      const auto& member = memberPair.second;
      if (member->type() == AttributeValueString::Node) {
        auto newMember = getOrGetDummy(points_, member->id, currElemId);
        rules[memberPair.first].emplace_back(newMember);
      } else if (member->type() == AttributeValueString::Way) {
        // can either be linestring or polygon
        if (polygons_.find(member->id) != polygons_.end()) {
          auto newMember = getOrGetDummy(polygons_, member->id, currElemId);
          rules[memberPair.first].emplace_back(newMember);
        } else {
          auto newMember = getOrGetDummy(lineStrings_, member->id, currElemId);
          rules[memberPair.first].emplace_back(newMember);
        }
      } else if (member->type() == AttributeValueString::Relation) {
        // could be lanelet or area. regulatory element is not allowed.
        auto type = member->attributes.find(AttributeNamesString::Type);
        if (type == member->attributes.end()) {
          parserError(currElemId,
                      "Relation refers to another relation "s + std::to_string(member->id) + " without a type tag!");
        } else if (type->second == AttributeValueString::Lanelet) {
          auto newMember = getOrGetDummy(lanelets_, member->id, currElemId);
          rules[memberPair.first].emplace_back(newMember);
        } else if (type->second == AttributeValueString::Multipolygon) {
          auto newMember = getOrGetDummy(areas_, member->id, currElemId);
          rules[memberPair.first].emplace_back(newMember);
        } else if (type->second == AttributeValueString::RegulatoryElement) {
          parserError(currElemId,
                      "Regulatory element refers to another "
                      "regulatory element. This is not "
                      "supported.");
        } else {
          parserError(currElemId, "Member of regulatory_element has unsupported type "s + type->second);
        }
      }
    }
    return rules;
  }

  std::vector<LineStrings3d> assembleBoundary(LineStrings3d lineStrings, Id id) {
    std::reverse(lineStrings.begin(), lineStrings.end());  // its easier to pop from a vector...
    std::vector<LineStrings3d> rings;
    rings.emplace_back(LineStrings3d());
    while (!lineStrings.empty()) {
      auto& currRing = rings.back();
      if (currRing.empty()) {
        currRing.push_back(lineStrings.back());
        lineStrings.pop_back();
      } else {
        const auto lastId = currRing.back().back().id();
        auto elem = std::find_if(lineStrings.rbegin(), lineStrings.rend(), [lastId](const auto& elem) {
          return elem.back().id() == lastId || elem.front().id() == lastId;
        });
        // we are unable to close the current ring
        if (elem == lineStrings.rend()) {
          parserError(id, "Could not complete boundary around linestring " + std::to_string(currRing.back().id()));
          rings.back() = LineStrings3d();
          continue;
        }
        // we found the matching next linestring. add it in the correct order
        auto newLineString = *elem;
        lineStrings.erase(std::next(elem).base());
        if (newLineString.back().id() == lastId) {
          newLineString = newLineString.invert();
        }
        currRing.push_back(newLineString);
      }

      // check if we closed the ring
      if (currRing.back().back().id() == currRing.front().front().id()) {
        // wohoo. Check the clockwise requirement.
        if (!isValid(currRing)) {
          reverse(currRing);
          if (!isValid(currRing)) {
            // most probably self-intersecting...
            parserError(id, "Failed to generate boundary (self-intersecting?)");
            rings.pop_back();
          }
        }
        rings.emplace_back(LineStrings3d());
      }
    }
    rings.pop_back();  // last ring will be empty or invalid
    return rings;
  }

  template <typename PrimT>
  PrimT getOrGetDummy(const typename std::unordered_map<Id, PrimT>& map, Id id, Id currentPrimitiveId) {
    try {
      return map.at(id);
    } catch (std::out_of_range&) {
      parserError(currentPrimitiveId, "Failed to get id "s + std::to_string(id) + " from map");
      return getDummy<PrimT>(id);
    }
  }

  void parserError(Id id, const std::string& what) {
    auto errstr = "Error parsing primitive "s + std::to_string(id) + ": " + what;
    errors_.push_back(errstr);
  }

  Errors errors_;
  LaneletLayer::Map lanelets_;
  AreaLayer::Map areas_;
  RegulatoryElementLayer::Map regulatoryElements_;
  PolygonLayer::Map polygons_;
  LineStringLayer::Map lineStrings_;
  PointLayer::Map points_;
};

template <typename MapT>
void registerIds(const MapT& map) {
  if (!map.empty()) {
    utils::registerId(map.rbegin()->first);
  }
}

void testAndPrintLocaleWarning(ErrorMessages& errors) {
  auto* decimalPoint = std::localeconv()->decimal_point;
  if (decimalPoint == nullptr || *decimalPoint != '.') {
    std::stringstream ss;
    ss << "Warning: Current decimal point of the C locale is set to \""
       << (decimalPoint == nullptr ? ' ' : *decimalPoint) << "\". The loaded map will have wrong coordinates!\n";
    errors.emplace_back(ss.str());
    std::cerr << errors.back();
  }
}
}  // namespace

std::unique_ptr<LaneletMap> OsmParser::parse(const std::string& filename, ErrorMessages& errors) const {
  // read xml
  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::ParseError("Errors occured while parsing osm file: "s + result.description());
  }
  osm::Errors osmReadErrors;
  testAndPrintLocaleWarning(osmReadErrors);
  auto file = lanelet::osm::read(doc, &osmReadErrors);
  auto map = fromOsmFile(file, errors);
  // make sure ids in the file are known to Lanelet2 id management.
  registerIds(file.nodes);
  registerIds(file.ways);
  registerIds(file.relations);
  errors = buildErrorMessage("Errors ocurred while parsing Lanelet Map:", utils::concatenate({osmReadErrors, errors}));
  return map;
}

std::unique_ptr<LaneletMap> OsmParser::fromOsmFile(const osm::File& file, ErrorMessages& errors) const {
  return FromFileLoader::loadMap(file, projector(), errors);
}
}  // namespace io_handlers
}  // namespace lanelet
