#include "io_handlers/OsmFile.h"
#include <lanelet2_core/utility/Utilities.h>
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
constexpr const char* Elevation = "ele";
constexpr const char* Action = "action";
constexpr const char* Delete = "delete";
}  // namespace keyword

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

class OsmFileWriter {
 public:
  static std::unique_ptr<pugi::xml_document> write(const File& osmFile) {
    OsmFileWriter osmIo;
    auto xml = std::make_unique<pugi::xml_document>();
    auto osmNode = xml->append_child(keyword::Osm);
    osmNode.append_attribute("version") = "0.6";
    osmNode.append_attribute("generator") = "lanelet2";
    osmIo.writeNodes(osmNode, osmFile.nodes);
    osmIo.writeWays(osmNode, osmFile.ways);
    osmIo.writeRelations(osmNode, osmFile.relations);
    return xml;
  }

 private:
  void writeAttributes(pugi::xml_node& elemNode, const Attributes& attributes) {
    for (const auto& attribute : attributes) {
      auto tagNode = elemNode.append_child(keyword::Tag);
      tagNode.append_attribute(keyword::Key) = attribute.first.c_str();
      tagNode.append_attribute(keyword::Value) = attribute.second.c_str();
    }
  }

  void writeNodes(pugi::xml_node& osmNode, const Nodes& nodes) {
    for (const auto& node : nodes) {
      auto xmlNode = osmNode.append_child(keyword::Node);
      xmlNode.append_attribute(keyword::Id) = LongLong(node.second.id);
      xmlNode.append_attribute(keyword::Lat) = node.second.point.lat;
      xmlNode.append_attribute(keyword::Lon) = node.second.point.lon;
      if (node.second.id > 0) {
        xmlNode.append_attribute(keyword::Version) = 1;
      }
      if (node.second.point.ele != 0.) {
        auto tagNode = xmlNode.append_child(keyword::Tag);
        tagNode.append_attribute(keyword::Key) = keyword::Elevation;
        tagNode.append_attribute(keyword::Value) = node.second.point.ele;
      }
      writeAttributes(xmlNode, node.second.attributes);
    }
  }

  void writeWays(pugi::xml_node& osmNode, const Ways& ways) {
    for (const auto& wayElem : ways) {
      const auto& way = wayElem.second;
      auto xmlNode = osmNode.append_child(keyword::Way);
      if (way.id > 0) {
        xmlNode.append_attribute(keyword::Version) = 1;
      }
      xmlNode.append_attribute(keyword::Id) = LongLong(way.id);
      for (const auto& node : way.nodes) {
        auto nd = xmlNode.append_child(keyword::Nd);
        nd.append_attribute(keyword::Ref) = LongLong(node->id);
      }
      writeAttributes(xmlNode, way.attributes);
    }
  }

  void writeRelations(pugi::xml_node& osmNode, const Relations& relations) {
    for (const auto& relationElem : relations) {
      const auto& relation = relationElem.second;
      auto xmlNode = osmNode.append_child(keyword::Relation);
      xmlNode.append_attribute(keyword::Id) = LongLong(relation.id);
      if (relation.id > 0) {
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
    file.nodes = osmParser.readNodes(osmNode);
    file.ways = osmParser.readWays(osmNode, file.nodes);
    file.relations = osmParser.readRelations(osmNode, file.nodes, file.ways);
    if (errors != nullptr) {
      *errors = osmParser.errors_;
    }
    return file;
  }

 private:
  Nodes readNodes(const pugi::xml_node& osmNode) {
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
    using UnresolvedRole = std::pair<std::string, Id>;
    using UnresolvedRoles = std::vector<UnresolvedRole>;
    using UnresolvedRolesPair = std::pair<Relation*, UnresolvedRoles>;
    std::vector<UnresolvedRolesPair> unresolvedRelations;     //!< relations referring to other relations
    for (auto node = osmNode.child(keyword::Relation); node;  // NOLINT
         node = node.next_sibling(keyword::Relation)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);

      // resolve members
      Roles roles;
      UnresolvedRoles unresolvedRoles;
      for (auto member = node.child(keyword::Member); member;  // NOLINT
           member = member.next_sibling(keyword::Member)) {
        Id memberId = member.attribute(keyword::Ref).as_llong();
        const std::string role = member.attribute(keyword::Role).value();
        const std::string type = member.attribute(keyword::Type).value();
        try {
          if (type == keyword::Node) {
            roles.insert(std::make_pair(role, &nodes.at(memberId)));
          } else if (type == keyword::Way) {
            roles.insert(std::make_pair(role, &ways.at(memberId)));
          } else if (type == keyword::Relation) {
            unresolvedRoles.push_back({role, memberId});
          }
        } catch (std::out_of_range&) {
          reportParseError(id, "Relation has nonexistent member " + std::to_string(memberId));
        }
      }
      auto inserted = relations.insert(Relations::value_type{id, Relation{id, attributes, roles}});
      if (!unresolvedRoles.empty()) {
        unresolvedRelations.emplace_back(&inserted.first->second, unresolvedRoles);
      }
    }

    // now resolve all unresolved roles that point to other relations
    for (const auto& unresolvedRelation : unresolvedRelations) {
      for (const auto& role : unresolvedRelation.second) {
        try {
          unresolvedRelation.first->members.insert(std::make_pair(role.first, &relations.at(role.second)));

        } catch (std::out_of_range&) {
          reportParseError(unresolvedRelation.first->id,
                           "Relation references nonexistent relation " + std::to_string(role.second));
        }
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
    try {
      for (const auto& member : lhs.members) {
        auto rhsmem = rhs.members.equal_range(member.first);
        while (rhsmem.first != rhsmem.second) {
          if (member.second->type() == rhsmem.first->second->type() && member.second->id == rhsmem.first->second->id) {
            break;
          }
          ++rhsmem.first;
        }
        if (rhsmem.first == rhsmem.second) {
          return false;
        }
      }
      return true;

    } catch (std::out_of_range&) {
      return false;
    }
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
