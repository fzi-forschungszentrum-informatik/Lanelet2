#pragma once
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/GPSPoint.h>

#include <deque>
#include <map>
#include <pugixml.hpp>
#include <string>
#include <utility>

namespace lanelet {
namespace osm {

struct Primitive;
using Attributes = std::map<std::string, std::string>;
using Role = std::pair<std::string, Primitive*>;
using Roles = std::deque<Role>;  // need iterator validity on push_back
using Errors = std::vector<std::string>;

//! Common abstract base class for all osm primitives. Provides id and attributes.
struct Primitive {
  Primitive() = default;
  Primitive(Primitive&& rhs) noexcept = default;  // NOLINT
  Primitive& operator=(Primitive&& rhs) noexcept = default;
  Primitive(const Primitive& rhs) = delete;
  Primitive& operator=(const Primitive& rhs) = delete;
  virtual ~Primitive() = default;
  Primitive(Id id, Attributes attributes) : id{id}, attributes{std::move(attributes)} {}
  virtual std::string type() = 0;

  Id id{0};
  Attributes attributes;
};

//! Osm node object
struct Node : public Primitive {
  Node() = default;
  Node(Id id, Attributes attributes, GPSPoint point) : Primitive{id, std::move(attributes)}, point{point} {}
  std::string type() override { return "node"; }
  GPSPoint point;
};

//! Osm way object
struct Way : public Primitive {
  Way() = default;
  Way(Id id, Attributes attributes, std::vector<Node*> nodes)
      : Primitive{id, std::move(attributes)}, nodes{std::move(nodes)} {}
  std::string type() override { return "way"; }
  std::vector<Node*> nodes;
};

//! Osm relation object
struct Relation : public Primitive {
  Relation() = default;
  Relation(Id id, Attributes attributes, Roles roles = Roles())
      : Primitive{id, std::move(attributes)}, members{std::move(roles)} {}
  std::string type() override { return "relation"; }
  Roles members;
};

using Nodes = std::map<Id, Node>;
using Ways = std::map<Id, Way>;
using Relations = std::map<Id, Relation>;

/**
 * @brief Intermediate representation of an osm file.
 *
 * Tries its best to cover the osm file specification.
 */
struct File {
  File() noexcept = default;
  File(File&& rhs) noexcept = default;  // NOLINT
  File& operator=(File&& rhs) noexcept = default;
  File(const File& rhs) = delete;
  File& operator=(const File& rhs) = delete;
  ~File() noexcept = default;

  Nodes nodes;
  Ways ways;
  Relations relations;
};

inline auto findRole(const Roles& roles, const std::string& roleName) {
  return std::find_if(roles.begin(), roles.end(), [&](auto& role) { return roleName == role.first; });
}

template <typename Func>
inline auto forEachMember(const Roles& roles, const std::string& roleName, Func&& f) {
  std::for_each(roles.begin(), roles.end(), [&](auto& role) {
    if (roleName == role.first) {
      f(role);
    };
  });
}

//! Reads an xml document into an osm file representation and optionally reports
//! parser errors.
File read(pugi::xml_document& node, lanelet::osm::Errors* errors = nullptr);

//! Creates an xml representation from an osm file representation. This is
//! guaranteed to work without errors.
std::unique_ptr<pugi::xml_document> write(const File& file);

inline bool operator==(const Node& lhs, const Node& rhs) { return lhs.id == rhs.id; }
bool operator==(const Way& lhs, const Way& rhs);
bool operator==(const Relation& lhs, const Relation& rhs);
bool operator==(const File& lhs, const File& rhs);
}  // namespace osm
}  // namespace lanelet
