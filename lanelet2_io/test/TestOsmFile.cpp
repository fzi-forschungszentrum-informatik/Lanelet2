#include <sstream>

#include "gtest/gtest.h"
#include "lanelet2_io/io_handlers/OsmFile.h"

using namespace lanelet::osm;

TEST(OsmFile, readWrite) {  // NOLINT
  File file;
  // clang-format off
  file.nodes.emplace(std::make_pair(-1, Node{-1, {{"key", "value"}, {"key2", "value2"}}, {12.3456789, 0001.2345678000, 9.0123}}));
  file.nodes.emplace(std::make_pair(-2, Node{-2, {}, {45.6789, 12.3, 4.5}}));
  file.nodes.emplace(std::make_pair(-3, Node{-3, {}, {67.8, 9.10, 11.0}}));
  file.ways.emplace(std::make_pair(-4, Way{-4, {{"wayKey", "wayValue"}}, {&file.nodes.at(-1), &file.nodes.at(-2)}}));
  file.ways.emplace(std::make_pair(-5, Way{-5, {{"wayKey", "wayValue"}}, {&file.nodes.at(-2), &file.nodes.at(-3)}}));
  file.relations.emplace(std::make_pair(-6,Relation{-6,{{"relKey", "relValue"}},{{"outer", &file.ways.at(-4)}, {"outer", &file.ways.at(-5)}, {"node", &file.nodes.at(-2)}}}));
  // clang-format on
  auto doc = write(file);
  auto file2 = read(*doc);

  EXPECT_EQ(file2.relations, file.relations);
  EXPECT_EQ(file2.ways, file.ways);
  EXPECT_EQ(file2, file);
  // Test upload (false by default)
  const auto osmNode = doc->first_child();
  const auto upload_attribute = osmNode.attributes().begin()->next_attribute();
  EXPECT_EQ(std::string(upload_attribute.name()), "upload");
  EXPECT_EQ(std::string(upload_attribute.value()), "false");
  // Test josm_format_elevation (false by default)
  EXPECT_DOUBLE_EQ(file2.nodes.at(-1).point.lat, 12.3456789);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-1).point.lon, 1.2345678);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-1).point.ele, 9.0123);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-2).point.lat, 45.6789);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-2).point.lon, 12.3);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-2).point.ele, 4.5);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-3).point.lat, 67.8);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-3).point.lon, 9.10);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-3).point.ele, 11.0);
}

TEST(OsmFile, readWriteJSON) {  // NOLINT
  File file;
  // clang-format off
  file.nodes.emplace(std::make_pair(-1, Node{-1, {{"key", "value"}, {"key2", "value2"}}, {12.3456789, 0001.2345678000, 9.0123}}));
  file.nodes.emplace(std::make_pair(-2, Node{-2, {}, {45.6789, 12.3, 4.5}}));
  file.nodes.emplace(std::make_pair(-3, Node{-3, {}, {67.8, 9.10, 11.0}}));
  file.ways.emplace(std::make_pair(-4, Way{-4, {{"wayKey", "wayValue"}}, {&file.nodes.at(-1), &file.nodes.at(-2)}}));
  file.ways.emplace(std::make_pair(-5, Way{-5, {{"wayKey", "wayValue"}}, {&file.nodes.at(-2), &file.nodes.at(-3)}}));
  file.relations.emplace(std::make_pair(-6,Relation{-6,{{"relKey", "relValue"}},{{"outer", &file.ways.at(-4)}, {"outer", &file.ways.at(-5)}, {"node", &file.nodes.at(-2)}}}));
  // clang-format on
  auto doc = write(file, { {"josm_upload", "true"}, {"josm_format_elevation", "true"} });
  auto file2 = read(*doc);

  EXPECT_EQ(file2.relations, file.relations);
  EXPECT_EQ(file2.ways, file.ways);
  EXPECT_EQ(file2, file);
  // Test upload
  const auto osmNode = doc->first_child();
  const auto upload_attribute = osmNode.attributes().begin()->next_attribute();
  EXPECT_EQ(std::string(upload_attribute.name()), "upload");
  EXPECT_EQ(std::string(upload_attribute.value()), "true");
  // Test josm_format_elevation
  EXPECT_DOUBLE_EQ(file2.nodes.at(-1).point.lat, 12.3456789);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-1).point.lon, 1.2345678);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-1).point.ele, 9.01);  // limited to 2 decimals
  EXPECT_DOUBLE_EQ(file2.nodes.at(-2).point.lat, 45.6789);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-2).point.lon, 12.3);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-2).point.ele, 4.5);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-3).point.lat, 67.8);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-3).point.lon, 9.10);
  EXPECT_DOUBLE_EQ(file2.nodes.at(-3).point.ele, 11.0);
}

TEST(OsmFile, readEmptyFile) {  // NOLINT
  pugi::xml_document doc;
  doc.load_string("");
  Errors errors;
  auto file = read(doc, &errors);
  EXPECT_TRUE(errors.empty());
}

TEST(OsmFile, readFileWithoutMapData) {  // NOLINT
  pugi::xml_document doc;
  doc.load_string("<osm></osm>");
  Errors errors;
  auto file = read(doc, &errors);
  EXPECT_TRUE(errors.empty());
}

TEST(OsmFile, readFileWithIncompleteMapData) {  // NOLINT
  pugi::xml_document doc;
  doc.load_string(R"(<osm><way id="1"><nd ref="2"/></way><node/></osm>)");
  Errors errors;
  auto file = read(doc, &errors);
  EXPECT_FALSE(errors.empty());
}

TEST(OsmFile, readMapWithIncompleteRoles) {  // NOLINT
  pugi::xml_document doc;
  doc.load_string(R"(<osm>
                    <node id="1" lat="1" lon="1"/>
                    <way id="1">
                      <nd ref="1"/>
                    </way>
                    <relation id="1">
                      <tag k="type" v="lanelet"/>
                      <member type="way" ref="2" role="left"/>
                      <member type="way" ref="1" role="right"/>
                    </relation>
                    <relation id="2">
                      <tag k="type" v="regulatory_element"/>
                      <member type="way" ref="1" role="somerole"/>
                      <member type="way" ref="2" role="nonexisting"/>
                      <member type="relation" ref="3" role="nonexisting"/>
                      <member type="relation" ref="4" role="nonexisting"/>
                      <member type="relation" ref="1" role="somerole2"/>
                      <member type="way" ref="1" role="somerole3"/>
                    </relation>
                    </osm>)");
  Errors errors;
  auto file = read(doc, &errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(file.nodes.size(), 1UL);
  EXPECT_EQ(file.ways.size(), 1UL);
  ASSERT_EQ(file.relations.size(), 2UL);
  ASSERT_NE(file.relations.find(2), file.relations.end());
  EXPECT_EQ(file.relations[1].members.size(), 1UL);
  EXPECT_EQ(file.relations[2].members.size(), 3UL);
  auto& members = file.relations[2].members;
  EXPECT_EQ(members[0].first, "somerole");
  EXPECT_EQ(members[0].second->id, 1L);
  EXPECT_EQ(members[1].first, "somerole2");
  EXPECT_EQ(members[2].first, "somerole3");
}
