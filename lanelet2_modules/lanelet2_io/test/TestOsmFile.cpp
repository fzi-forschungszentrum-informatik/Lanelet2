#include "gtest/gtest.h"
#include "io_handlers/OsmFile.h"

using namespace lanelet::osm;

TEST(OsmFile, readWrite) {  // NOLINT
  File file;
  // clang-format off
  file.nodes.emplace(std::make_pair(-1, Node{-1, {{"key", "value"}, {"key2", "value2"}}, {49, 8, 1}}));
  file.nodes.emplace(std::make_pair(-2, Node{-2, {}, {50, 8, 0}}));
  file.nodes.emplace(std::make_pair(-3, Node{-3, {}, {50, 8, 0}}));
  file.ways.emplace(std::make_pair(-4, Way{-4, {{"wayKey", "wayValue"}}, {&file.nodes.at(-1), &file.nodes.at(-2)}}));
  file.ways.emplace(std::make_pair(-5, Way{-5, {{"wayKey", "wayValue"}}, {&file.nodes.at(-2), &file.nodes.at(-3)}}));
  file.relations.emplace(std::make_pair(-6,Relation{-6,{{"relKey", "relValue"}},{{"outer", &file.ways.at(-4)}, {"outer", &file.ways.at(-5)}, {"node", &file.nodes.at(-2)}}}));
  // clang-format on
  auto doc = write(file);

  auto file2 = read(*doc);

  EXPECT_EQ(file2.relations, file.relations);
  EXPECT_EQ(file2.ways, file.ways);
  EXPECT_EQ(file2, file);
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
  doc.load_string(R"(<osm><way id="1"><nd ref="2"/></way></osm>)");
  Errors errors;
  auto file = read(doc, &errors);
  EXPECT_FALSE(errors.empty());
}
