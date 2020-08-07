#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <cstdio>

#include "TestSetup.h"
#include "lanelet2_io/Io.h"
#include "lanelet2_io/io_handlers/OsmHandler.h"

using namespace lanelet;

template <typename T, typename TT>
T writeAndLoad(const T& value, TT(LaneletMapLayers::*layer)) {
  LaneletMap llmap;
  llmap.add(value);

  ErrorMessages errsWrite;
  ErrorMessages errsLoad;
  auto projector = defaultProjection(Origin({0, 0, 0}));
  io_handlers::OsmParser parser(projector);
  io_handlers::OsmWriter writer(projector);
  auto file = writer.toOsmFile(llmap, errsWrite);
  auto loadedMap = parser.fromOsmFile(*file, errsLoad);
  EXPECT_TRUE(errsWrite.empty());
  EXPECT_TRUE(errsLoad.empty());
  auto& valueLayer = llmap.*layer;
  EXPECT_EQ(1ul, valueLayer.size());
  return valueLayer.get(utils::getId(value));
}

TEST(OsmHandler, writeAndLoadMapWithOneLanelet) {  // NOLINT
  auto num = 1;
  Lanelet llt = test_setup::setUpLanelet(num);
  auto genRegelem = test_setup::setUpGenericRegulatoryElement(num);
  std::dynamic_pointer_cast<lanelet::GenericRegulatoryElement>(genRegelem)->addParameter("lanelet", llt);
  llt.addRegulatoryElement(genRegelem);
  auto lltLoad = writeAndLoad(llt, &LaneletMap::laneletLayer);
  EXPECT_EQ(llt.inverted(), lltLoad.inverted());
  EXPECT_EQ(*llt.constData(), *lltLoad.constData());
  EXPECT_EQ(lltLoad.regulatoryElementsAs<GenericRegulatoryElement>()
                .front()
                ->getParameters<lanelet::ConstLanelet>("lanelet")
                .front(),
            lltLoad);
}

TEST(OsmHandler, writeAndLoadMapWithCenterlineLanelet) {  // NOLINT
  auto num = 1;
  Lanelet llt = test_setup::setUpLanelet(num);
  LineString3d centerline = test_setup::setUpLineString(num).invert();
  llt.setCenterline(centerline);
  auto lltLoad = writeAndLoad(llt, &LaneletMap::laneletLayer);
  EXPECT_EQ(lltLoad.centerline().constData(), centerline.constData());
  EXPECT_EQ(lltLoad.centerline().inverted(), centerline.inverted());
}

TEST(OsmHandler, writeAndLoadMapWithPolygon) {  // NOLINT
  auto num = 1;
  Polygon3d poly{test_setup::setUpLineString(num)};
  auto polyLoad = writeAndLoad(poly, &LaneletMap::polygonLayer);
  EXPECT_EQ(poly.id(), polyLoad.id());
}

TEST(OsmHandler, writeAndLoadMapWithOneArea) {  // NOLINT
  auto map = std::make_unique<lanelet::LaneletMap>();
  auto num = 1;
  auto ar = test_setup::setUpArea(num);
  auto genRegelem = test_setup::setUpGenericRegulatoryElement(num);
  std::dynamic_pointer_cast<lanelet::GenericRegulatoryElement>(genRegelem)->addParameter("area", ar);
  ar.addRegulatoryElement(genRegelem);
  auto arLoad = writeAndLoad(ar, &LaneletMap::areaLayer);
  EXPECT_EQ(*ar.constData(), *arLoad.constData());
  EXPECT_EQ(arLoad.regulatoryElementsAs<GenericRegulatoryElement>()
                .front()
                ->getParameters<lanelet::ConstArea>("area")
                .front(),
            arLoad);
}

TEST(OsmHandler, writeAndLoadMapWithRegElem) {  // NOLINT
  auto map = std::make_unique<lanelet::LaneletMap>();
  auto num = 1;
  auto regElem = test_setup::setUpRegulatoryElement(num);
  auto rLoad = writeAndLoad(lanelet::RegulatoryElementPtr(regElem), &LaneletMap::regulatoryElementLayer);
  EXPECT_EQ(*regElem->constData(), *rLoad->constData());
}

TEST(OsmHandler, writeMapWithIncompleteRegelem) {  // NOLINT
  auto num = 1;
  auto regElem = test_setup::setUpRegulatoryElement(num);
  lanelet::LaneletMap map({}, {}, {{regElem->id(), regElem}}, {}, {}, {});
  ErrorMessages errsWrite;
  auto projector = defaultProjection(Origin({0, 0, 0}));
  auto file = io_handlers::OsmWriter(projector).toOsmFile(map, errsWrite);
  EXPECT_GT(errsWrite.size(), 0ul);
}

TEST(OsmHandler, writeMapWithIncompleteLanelet) {  // NOLINT
  auto num = 1;
  auto llt = test_setup::setUpLanelet(num);
  lanelet::LaneletMap map({{llt.id(), llt}}, {}, {}, {}, {}, {});
  ErrorMessages errsWrite;
  auto projector = defaultProjection(Origin({0, 0, 0}));
  auto file = io_handlers::OsmWriter(projector).toOsmFile(map, errsWrite);
  EXPECT_GT(errsWrite.size(), 0ul);
}

TEST(OsmHandler, writeMapWithLaneletAndAreaToFile) {  // NOLINT
  auto map = std::make_unique<lanelet::LaneletMap>();
  auto num = 1;
  auto ar = test_setup::setUpArea(num);
  auto ll = test_setup::setUpLanelet(num);
  map->add(ar);
  map->add(ll);
  lanelet::test_setup::Tempfile file("file.osm");
  Origin origin({49, 8.4, 0});
  write(file.get().string(), *map, origin);
  EXPECT_NO_THROW(load(file.get().string(), origin));  // NOLINT
}
