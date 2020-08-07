#include "lanelet2_io/io_handlers/OsmFile.h"

#include <lanelet2_core/utility/Utilities.h>

#include <boost/format.hpp>
#include <iostream>

namespace lanelet {
namespace osm {
namespace {
using LongLong = long long;  // NOLINT

namespace keyword {
constexpr const char* Osm = "osm";
constexpr const char* Tag = "tag";
constexpr const char* Key = "k";
constexpr const char* Value = "v";
constexpr const char* Node = "node";
constexpr const char* Way = "way";
constexpr const char* Relation = "relation";
constexpr const char* Member = "member";
constexpr const char* Role = "role";
constexpr const char* Type = "type";
constexpr const char* Nd = "nd";
constexpr const char* Ref = "ref";
constexpr const char* Id = "id";
constexpr const char* Lat = "lat";
constexpr const char* Lon = "lon";
constexpr const char* Version = "version";
constexpr const char* Visible = "visible";
constexpr const char* Elevation = "ele";
constexpr const char* Action = "action";
constexpr const char* Delete = "delete";
}  // namespace keyword

struct UnresolvedRole {
  Id relation{};
  Id referencedRelation{};
  Primitive** location{};
};

Attributes tags(const pugi::xml_node& node) {
  Attributes attributes;
  for (auto tag = node.child(keyword::Tag); tag;  // NOLINT
       tag = tag.next_sibling(keyword::Tag)) {
    if (std::string(tag.attribute(keyword::Key).value()) == keyword::Elevation) {
      continue;
    }
    attributes[tag.attribute(keyword::Key).value()] = tag.attribute(keyword::Value).value();
  }
  return attributes;
}

bool isDeleted(const pugi::xml_node& node) {
  auto action = node.attribute(keyword::Action);
  return action && std::string(action.value()) == keyword::Delete;  // NOLINT
}

std::string toJosmStyle(double d) {
  std::string str = boost::str(boost::format{"%12.11f"} % d);
  str.erase(str.find_last_not_of('0') + 1, std::string::npos);
  str.erase(str.find_last_not_of('.') + 1, std::string::npos);
  return str;
}

void removeAndFixPlaceholders(Primitive** toRemove, Roles& fromRoles, std::vector<UnresolvedRole>& placeholders) {
  // find other placeholders that we have to fix
  auto remIt =
      std::find_if(fromRoles.begin(), fromRoles.end(), [&](const Role& role) { return &role.second == toRemove; });
  assert(remIt != fromRoles.end());
  std::vector<std::pair<size_t, Primitive**>> placeholderLocations;
  for (auto it = fromRoles.begin(); it != fromRoles.end(); ++it) {
    if (it->second == nullptr && remIt != it) {
      auto idx = std::distance(fromRoles.begin(), it);
      placeholderLocations.emplace_back(it > remIt ? idx - 1 : idx, &it->second);
    }
  }
  fromRoles.erase(remIt);
  if (placeholderLocations.empty()) {
    return;  // nothing to update
  }
  // get the new locations
  std::map<Primitive**, Primitive**> newLocations;
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

class OsmFileWriter {
 public:
  static std::unique_ptr<pugi::xml_document> write(const File& osmFile) {
    auto xml = std::make_unique<pugi::xml_document>();
    auto osmNode = xml->append_child(keyword::Osm);
    osmNode.append_attribute("version") = "0.6";
    osmNode.append_attribute("generator") = "lanelet2";
    lanelet::osm::OsmFileWriter::writeNodes(osmNode, osmFile.nodes);
    lanelet::osm::OsmFileWriter::writeWays(osmNode, osmFile.ways);
    lanelet::osm::OsmFileWriter::writeRelations(osmNode, osmFile.relations);
    return xml;
  }

 private:
  static void writeAttributes(pugi::xml_node& elemNode, const Attributes& attributes) {
    for (const auto& attribute : attributes) {
      auto tagNode = elemNode.append_child(keyword::Tag);
      tagNode.append_attribute(keyword::Key) = attribute.first.c_str();
      tagNode.append_attribute(keyword::Value) = attribute.second.c_str();
    }
  }

  static void writeNodes(pugi::xml_node& osmNode, const Nodes& nodes) {
    for (const auto& node : nodes) {
      auto xmlNode = osmNode.append_child(keyword::Node);
      xmlNode.append_attribute(keyword::Id) = LongLong(node.second.id);
      if (node.second.id > 0) {
        xmlNode.append_attribute(keyword::Visible) = "true";
        xmlNode.append_attribute(keyword::Version) = 1;
      }
      xmlNode.append_attribute(keyword::Lat) = toJosmStyle(node.second.point.lat).c_str();
      xmlNode.append_attribute(keyword::Lon) = toJosmStyle(node.second.point.lon).c_str();

      if (node.second.point.ele != 0.) {
        auto tagNode = xmlNode.append_child(keyword::Tag);
        tagNode.append_attribute(keyword::Key) = keyword::Elevation;
        tagNode.append_attribute(keyword::Value) = node.second.point.ele;
      }
      writeAttributes(xmlNode, node.second.attributes);
    }
  }

  static void writeWays(pugi::xml_node& osmNode, const Ways& ways) {
    for (const auto& wayElem : ways) {
      const auto& way = wayElem.second;
      auto xmlNode = osmNode.append_child(keyword::Way);
      xmlNode.append_attribute(keyword::Id) = LongLong(way.id);
      if (way.id > 0) {
        xmlNode.append_attribute(keyword::Visible) = "true";
        xmlNode.append_attribute(keyword::Version) = 1;
      }
      for (const auto& node : way.nodes) {
        auto nd = xmlNode.append_child(keyword::Nd);
        nd.append_attribute(keyword::Ref) = LongLong(node->id);
      }
      writeAttributes(xmlNode, way.attributes);
    }
  }

  static void writeRelations(pugi::xml_node& osmNode, const Relations& relations) {
    for (const auto& relationElem : relations) {
      const auto& relation = relationElem.second;
      auto xmlNode = osmNode.append_child(keyword::Relation);
      xmlNode.append_attribute(keyword::Id) = LongLong(relation.id);
      if (relation.id > 0) {
        xmlNode.append_attribute(keyword::Visible) = "true";
        xmlNode.append_attribute(keyword::Version) = 1;
      }
      for (const auto& role : relation.members) {
        auto xmlMember = xmlNode.append_child(keyword::Member);
        auto type = role.second->type();
        xmlMember.append_attribute(keyword::Type) = type.c_str();
        xmlMember.append_attribute(keyword::Ref) = LongLong(role.second->id);
        xmlMember.append_attribute(keyword::Role) = role.first.c_str();
      }
      writeAttributes(xmlNode, relation.attributes);
    }
  }
};

class OsmFileParser {
 public:
  static File read(const pugi::xml_node& fileNode, Errors* errors = nullptr) {
    OsmFileParser osmParser;
    File file;
    auto osmNode = fileNode.child(keyword::Osm);
    file.nodes = lanelet::osm::OsmFileParser::readNodes(osmNode);
    file.ways = osmParser.readWays(osmNode, file.nodes);
    file.relations = osmParser.readRelations(osmNode, file.nodes, file.ways);
    if (errors != nullptr) {
      *errors = osmParser.errors_;
    }
    return file;
  }

 private:
  static Nodes readNodes(const pugi::xml_node& osmNode) {
    Nodes nodes;
    for (auto node = osmNode.child(keyword::Node); node;  // NOLINT
         node = node.next_sibling(keyword::Node)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      const auto lat = node.attribute(keyword::Lat).as_double(0.);
      const auto lon = node.attribute(keyword::Lon).as_double(0.);
      const auto ele = node.find_child_by_attribute(keyword::Tag, keyword::Key, keyword::Elevation)
                           .attribute(keyword::Value)
                           .as_double(0.);
      nodes[id] = Node{id, attributes, {lat, lon, ele}};
    }
    return nodes;
  }

  Ways readWays(const pugi::xml_node& osmNode, Nodes& nodes) {
    Ways ways;
    for (auto node = osmNode.child(keyword::Way); node;  // NOLINT
         node = node.next_sibling(keyword::Way)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      const auto nodeIds = [&node] {
        Ids ids;
        for (auto refNode = node.child(keyword::Nd); refNode;  // NOLINT
             refNode = refNode.next_sibling(keyword::Nd)) {
          ids.push_back(refNode.attribute(keyword::Ref).as_llong());
        }
        return ids;
      }();
      std::vector<Node*> wayNodes;
      try {
        wayNodes = utils::transform(nodeIds, [&nodes](const auto& elem) { return &nodes.at(elem); });
      } catch (std::out_of_range&) {
        reportParseError(id, "Way references nonexisting points");
      }
      ways[id] = Way{id, attributes, wayNodes};
    }
    return ways;
  }

  Relations readRelations(const pugi::xml_node& osmNode, Nodes& nodes, Ways& ways) {
    Relations relations;
    // Two-pass approach: We can resolve all roles except where relations reference relations. We insert a dummy nullptr
    // and resolve that later on.
    std::vector<UnresolvedRole> unresolvedRoles;
    for (auto node = osmNode.child(keyword::Relation); node;  // NOLINT
         node = node.next_sibling(keyword::Relation)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      auto& relation = relations.emplace(id, Relation{id, attributes, {}}).first->second;

      // resolve members
      auto& roles = relation.members;
      for (auto member = node.child(keyword::Member); member;  // NOLINT
           member = member.next_sibling(keyword::Member)) {
        Id memberId = member.attribute(keyword::Ref).as_llong();
        const std::string role = member.attribute(keyword::Role).value();
        const std::string type = member.attribute(keyword::Type).value();
        try {
          if (type == keyword::Node) {
            roles.emplace_back(role, &nodes.at(memberId));
          } else if (type == keyword::Way) {
            roles.emplace_back(role, &ways.at(memberId));
          } else if (type == keyword::Relation) {
            // insert a placeholder and store a pointer to it for the second pass
            roles.emplace_back(role, nullptr);
            unresolvedRoles.push_back(UnresolvedRole{id, memberId, &roles.back().second});
          }
        } catch (std::out_of_range&) {
          reportParseError(id, "Relation has nonexistent member " + std::to_string(memberId));
        }
      }
    }

    // now resolve all unresolved roles that point to other relations
    for (const auto& unresolvedRole : unresolvedRoles) {
      try {
        assert(*unresolvedRole.location == nullptr);
        *unresolvedRole.location = &relations.at(unresolvedRole.referencedRelation);
      } catch (std::out_of_range&) {
        reportParseError(unresolvedRole.relation, "Relation references nonexistent relation " +
                                                      std::to_string(unresolvedRole.referencedRelation));
        // now it gets ugly: find placeholder and remove it. Fix all other placeholders because the pointers are
        // invalidated after moving. This is inefficent, but its the fault of the guy loading an invalid map, not ours.
        auto& relation = relations.at(unresolvedRole.relation);
        removeAndFixPlaceholders(unresolvedRole.location, relation.members, unresolvedRoles);
      }
    }
    return relations;
  }

  OsmFileParser() = default;
  void reportParseError(Id id, const std::string& what) {
    auto errstr = "Error reading primitive with id " + std::to_string(id) + " from file: " + what;
    errors_.push_back(errstr);
  }
  Errors errors_;
};
}  // namespace

bool operator==(const Way& lhs, const Way& rhs) {
  auto nodesEqual = [&lhs, &rhs]() {
    for (auto i = 0u; i < lhs.nodes.size(); i++) {
      if (lhs.nodes[i]->id != rhs.nodes[i]->id) {
        return false;
      }
    }
    return true;
  };
  return lhs.id == rhs.id && lhs.nodes.size() == rhs.nodes.size() && nodesEqual();
}

bool operator==(const Relation& lhs, const Relation& rhs) {
  auto membersEqual = [&lhs, &rhs]() {
    for (auto itL = lhs.members.begin(), itR = rhs.members.begin(); itL != lhs.members.end(); ++itL, ++itR) {
      if (itL->second->type() != itR->second->type() && itL->second->id != itR->second->id) {
        return false;
      }
    }
    return true;
  };
  return lhs.id == rhs.id && lhs.members.size() == rhs.members.size() && membersEqual();
}

bool operator==(const File& lhs, const File& rhs) {
  return lhs.nodes == rhs.nodes && lhs.ways == rhs.ways && lhs.relations == rhs.relations;
}

File read(pugi::xml_document& node, Errors* errors) { return OsmFileParser::read(node, errors); }

std::unique_ptr<pugi::xml_document> write(const File& file) { return OsmFileWriter::write(file); }
}  // namespace osm
}  // namespace lanelet
