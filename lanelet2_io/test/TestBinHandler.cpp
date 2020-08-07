#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <cstdio>
#include <fstream>

#include "TestSetup.h"
#include "gtest/gtest.h"
#include "lanelet2_io/Io.h"
#include "lanelet2_io/io_handlers/Serialize.h"

using namespace lanelet;

struct MyStuff {
  int i{};
};

template <typename T>
T writeAndLoad(const T& t) {
  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << t;
  T tout;
  boost::archive::binary_iarchive ia(ss);
  ia >> tout;
  return tout;
}

TEST(Serialize, Attribute) {  // NOLINT
  lanelet::AttributeMap am{{"key", "value"}, {"key1", "value1"}};
  EXPECT_EQ(am, writeAndLoad(am));
}

class SerializeTest : public ::testing::Test {
 public:
  int id{0};
  lanelet::AttributeMap attr{{"key", "value"}};
  lanelet::Point3d p1{test_setup::setUpPoint(id)};
  lanelet::Point3d p2{test_setup::setUpPoint(id, 2)};
  lanelet::LineString3d ls1{test_setup::setUpLineString(id)};
  lanelet::LineString3d ls2{test_setup::setUpLineString(id)};
  lanelet::RegulatoryElementPtr regelem{test_setup::setUpRegulatoryElement(id)};
  lanelet::RegulatoryElementPtr genericRegelem{test_setup::setUpGenericRegulatoryElement(id)};
  lanelet::Lanelet llt{test_setup::setUpLanelet(id)};
  lanelet::Area ar{test_setup::setUpArea(id)};
};

TEST_F(SerializeTest, Point) {  // NOLINT
  auto pLoad = writeAndLoad(p1);
  EXPECT_EQ(*p1.constData(), *pLoad.constData());
}

TEST_F(SerializeTest, Regelem) {  // NOLINT
  auto rLoad = writeAndLoad(lanelet::RegulatoryElementPtr(regelem));
  EXPECT_EQ(*regelem->constData(), *rLoad->constData());
}

TEST_F(SerializeTest, Lanelet) {  // NOLINT
  std::dynamic_pointer_cast<lanelet::GenericRegulatoryElement>(genericRegelem)->addParameter("lanelet", llt);
  llt.addRegulatoryElement(genericRegelem);
  auto lltLoad = writeAndLoad(llt);
  EXPECT_EQ(llt.inverted(), lltLoad.inverted());
  EXPECT_EQ(*llt.constData(), *lltLoad.constData());
  EXPECT_EQ(lltLoad.regulatoryElementsAs<GenericRegulatoryElement>()
                .front()
                ->getParameters<lanelet::ConstLanelet>("lanelet")
                .front(),
            lltLoad);
}

TEST(OsmHandler, LaneletWithCenterline) {  // NOLINT
  auto num = 1;
  Lanelet llt = test_setup::setUpLanelet(num);
  LineString3d centerline = test_setup::setUpLineString(num).invert();
  llt.setCenterline(centerline);
  auto lltLoad = writeAndLoad(llt);
  EXPECT_TRUE(lltLoad.hasCustomCenterline());
  EXPECT_TRUE(*lltLoad.centerline().constData() == *centerline.constData());
  EXPECT_EQ(lltLoad.centerline().inverted(), centerline.inverted());
}

TEST_F(SerializeTest, Area) {  // NOLINT
  std::dynamic_pointer_cast<lanelet::GenericRegulatoryElement>(genericRegelem)->addParameter("area", ar);
  ar.addRegulatoryElement(genericRegelem);
  auto arLoad = writeAndLoad(ar);
  EXPECT_EQ(*ar.constData(), *arLoad.constData());
  EXPECT_EQ(arLoad.regulatoryElementsAs<GenericRegulatoryElement>()
                .front()
                ->getParameters<lanelet::ConstArea>("area")
                .front(),
            arLoad);
}

TEST_F(SerializeTest, LaneletMap) {  // NOLINT
  std::dynamic_pointer_cast<lanelet::GenericRegulatoryElement>(genericRegelem)->addParameter("lanelet", llt);
  auto map = lanelet::utils::createMap({llt});
  auto mapLoad = writeAndLoad(*map);
  EXPECT_EQ(*map, mapLoad);
}

TEST(BinHandler, extension) {  // NOLINT
  lanelet::test_setup::Tempfile t("file.bin");
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::write(t.get().string(), *map);

  auto mapLoad = lanelet::load(t.get().string());
}

TEST(BinHandler, explicitIO) {  // NOLINT
  lanelet::test_setup::Tempfile t("file.bin");
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::write(t.get().string(), *map, "bin_handler");

  auto mapLoad = lanelet::load(t.get().string(), "bin_handler");
}

TEST(BinHandler, fullMap) {
  Origin origin({49, 8.4, 0});
  std::string filenameIn = "../../lanelet2_maps/res/mapping_example.osm";
  auto map = lanelet::load(filenameIn, origin);
  auto llt = map->laneletLayer.find(44968);
  writeAndLoad(*llt);
}
