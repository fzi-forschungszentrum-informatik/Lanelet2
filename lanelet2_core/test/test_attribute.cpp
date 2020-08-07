#include <gtest/gtest.h>

#include "lanelet2_core/Attribute.h"
#include "lanelet2_core/Exceptions.h"
#include "lanelet2_core/utility/Units.h"

using namespace std::string_literals;
using namespace lanelet;

TEST(Attribute, string) {  // NOLINT
  Attribute attr("bla");
  EXPECT_EQ("bla"s, attr.value());
}

TEST(Attribute, getSet) {  // NOLINT
  Attribute attr("bla");
  attr.setValue("bla2");
  EXPECT_EQ("bla2"s, attr.value());
}

TEST(Attribute, boolConversion) {  // NOLINT
  Attribute attr("yes");
  EXPECT_TRUE(*attr.asBool());
  EXPECT_TRUE(*attr.asBool());
  attr.setValue("no");
  EXPECT_FALSE(*attr.asBool());
  EXPECT_FALSE(*attr.asBool());

  // other possible values
  EXPECT_TRUE(*Attribute("true").asBool());
  EXPECT_TRUE(*Attribute("1").asBool());
  EXPECT_FALSE(*Attribute("false").asBool());
  EXPECT_FALSE(*Attribute("0").asBool());
}

TEST(Attribute, intConversion) {  // NOLINT
  Attribute attr(0);
  EXPECT_EQ(0., *attr.asInt());
  EXPECT_EQ(0., *attr.asInt());  // twice to test the cache
  EXPECT_FALSE(Attribute("0a").asInt());
  EXPECT_FALSE(Attribute(0.1).asInt());
}

TEST(Attribute, idConversion) {  // NOLINT
  Attribute attrMax(std::to_string(std::numeric_limits<Id>::max()));
  EXPECT_EQ(std::numeric_limits<Id>::max(), *attrMax.asId());

  Attribute attrMin(std::to_string(std::numeric_limits<Id>::min()));
  EXPECT_EQ(std::numeric_limits<Id>::min(), *attrMin.asId());

  EXPECT_FALSE(Attribute("0a").asId());
  EXPECT_FALSE(Attribute("0.1").asId());
  EXPECT_FALSE(Attribute(" 0").asId());
}

TEST(Attribute, doubleConversion) {  // NOLINT
  EXPECT_DOUBLE_EQ(0.1, *Attribute(0.1).asDouble());
  EXPECT_FALSE(Attribute("0a").asId());
}

TEST(Attribute, velocityConversion) {  // NOLINT
  using namespace units;
  using namespace units::literals;
  EXPECT_DOUBLE_EQ(0.1, KmHQuantity(*Attribute("0.1").asVelocity()).value());
  EXPECT_DOUBLE_EQ(0.1, Attribute("0.1 mps").asVelocity()->value());
  EXPECT_DOUBLE_EQ(0.1, Attribute("0.1mps").asVelocity()->value());
  EXPECT_DOUBLE_EQ(0.1, KmHQuantity(*Attribute("0.1 kmh").asVelocity()).value());
  EXPECT_DOUBLE_EQ(0.1, KmHQuantity(*Attribute("0.1 km/h").asVelocity()).value());
  EXPECT_DOUBLE_EQ(0.1, MPHQuantity(*Attribute("0.1 m/h").asVelocity()).value());
  EXPECT_DOUBLE_EQ(0.1, KmHQuantity(*Attribute(0.1_kmh).asVelocity()).value());
  EXPECT_DOUBLE_EQ(0.1, Velocity(*Attribute(0.1_mps).asVelocity()).value());
  EXPECT_FALSE(Attribute("0a").asVelocity());
}

TEST(Attribute, caching) {  // NOLINT
  Attribute attr("0.1");
  EXPECT_DOUBLE_EQ(*attr.asDouble(), 0.1);
  EXPECT_FALSE(attr.asId());
}

TEST(Attribute, as) {  // NOLINT
  Attribute attr("0");
  EXPECT_EQ(0, *attr.as<double>());
  EXPECT_EQ(0, *attr.as<Optional<double>>());
  EXPECT_EQ(0, *attr.as<Optional<int>>());
  EXPECT_EQ("0", *attr.as<std::string>());
  EXPECT_EQ("0", *attr.as<Optional<std::string>>());
  auto opt = attr.as<const char*>();
  EXPECT_EQ(std::string("0"), *opt);
  opt = attr.as<Optional<const char*>>();
  EXPECT_EQ(std::string("0"), *opt);
  EXPECT_EQ(0, *attr.as<Id>());
}

TEST(AttributeMap, enumAccess) {  // NOLINT
  AttributeMap attr;
  attr[AttributeName::Type] = "test";
  EXPECT_EQ(attr[AttributeName::Type], "test");
}

TEST(AttributeMap, normalAccess) {  // NOLINT
  AttributeMap attr;
  attr[AttributeNamesString::Type] = "test";
  EXPECT_EQ(attr[AttributeName::Type], "test");
}
