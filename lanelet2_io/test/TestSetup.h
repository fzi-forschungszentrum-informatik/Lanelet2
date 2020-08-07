#pragma once
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Exceptions.h>

#include <boost/filesystem.hpp>
#include <fstream>

namespace fs = boost::filesystem;

namespace lanelet {
inline bool operator==(const PointData& lhs, const PointData& rhs) {
  return lhs.id == rhs.id && lhs.attributes == rhs.attributes && lhs.point == rhs.point;
}

inline bool operator==(const LineStringData& lhs, const LineStringData& rhs) {
  return rhs.id == lhs.id && rhs.attributes == lhs.attributes && rhs.size() == lhs.size() &&
         std::equal(rhs.begin(false), rhs.end(false), lhs.begin(false),
                    [](auto& p1, auto& p2) { return *p1.constData() == *p2.constData(); });
}
inline bool operator==(const RegulatoryElementData& rhs, const RegulatoryElementData& lhs) {
  return rhs.id == lhs.id && lhs.attributes == rhs.attributes && rhs.parameters.size() == rhs.parameters.size();
}
inline bool operator==(const LaneletData& lhs, const LaneletData& rhs) {
  auto rhsRegelem = rhs.regulatoryElements();
  auto lhsRegelem = lhs.regulatoryElements();
  return rhs.id == lhs.id && rhs.attributes == lhs.attributes &&
         *lhs.leftBound().constData() == *rhs.leftBound().constData() &&
         *lhs.rightBound().constData() == *rhs.rightBound().constData() && lhsRegelem.size() == lhsRegelem.size() &&
         std::equal(rhsRegelem.begin(), rhsRegelem.end(), lhsRegelem.begin(),
                    [](auto& r1, auto& r2) { return *r1->constData() == *r2->constData(); });
}

inline bool operator==(const AreaData& lhs, const AreaData& rhs) {
  auto rhsRegelem = rhs.regulatoryElements();
  auto lhsRegelem = lhs.regulatoryElements();
  auto rhsOuter = rhs.outerBound();
  auto lhsOuter = lhs.outerBound();
  return rhs.id == lhs.id && rhs.attributes == lhs.attributes && rhsOuter.size() == rhsOuter.size() &&
         std::equal(rhsOuter.begin(), rhsOuter.end(), lhsOuter.begin(),
                    [](auto& ls1, auto& ls2) { return *ls1.constData() == *ls2.constData(); }) &&
         lhs.innerBounds().size() == rhs.innerBounds().size() && lhsRegelem.size() == lhsRegelem.size() &&
         std::equal(rhsRegelem.begin(), rhsRegelem.end(), lhsRegelem.begin(),
                    [](auto& r1, auto& r2) { return *r1->constData() == *r2->constData(); });
}

template <typename T>
inline bool operator==(const PrimitiveLayer<T>& rhs, const PrimitiveLayer<T>& lhs) {
  return rhs.size() == lhs.size() && std::all_of(rhs.begin(), rhs.end(), [&lhs](auto& v1) {
           return *v1.constData() == *lhs.find(v1.id())->constData();
         });
}

template <>
inline bool operator==<RegulatoryElementPtr>(const PrimitiveLayer<RegulatoryElementPtr>& rhs,
                                             const PrimitiveLayer<RegulatoryElementPtr>& lhs) {
  return rhs.size() == lhs.size() && std::all_of(rhs.begin(), rhs.end(), [&lhs](auto& v1) {
           return *v1->constData() == *(*lhs.find(v1->id()))->constData();
         });
}

inline bool operator==(const LaneletMap& rhs, const LaneletMap& lhs) {
  return lhs.pointLayer == rhs.pointLayer && lhs.lineStringLayer == rhs.lineStringLayer &&
         lhs.polygonLayer == rhs.polygonLayer && lhs.regulatoryElementLayer == rhs.regulatoryElementLayer &&
         lhs.areaLayer == rhs.areaLayer && lhs.laneletLayer == rhs.laneletLayer;
}

namespace test_setup {
inline Id getId(int& num) { return -(num++); }

inline Point3d setUpPoint(int& num, int xOffset = 0, const std::string& type = AttributeValueString::Start) {
  return Point3d(getId(num), xOffset, num, 1, AttributeMap{{AttributeNamesString::Type, type}});
}

inline LineString3d setUpLineString(int& num, const std::string& type = AttributeValueString::Curbstone) {
  return LineString3d(getId(num), {setUpPoint(num, 0), setUpPoint(num, 1)},
                      AttributeMap{{AttributeNamesString::Type, type}});
}

inline RegulatoryElementPtr setUpRegulatoryElement(int& num) {
  RuleParameterMap map{{RoleNameString::Refers, {setUpLineString(num, AttributeValueString::TrafficLight)}},
                       {RoleNameString::RefLine, {setUpLineString(num, AttributeValueString::StopLine)}}};
  return RegulatoryElementFactory::create(TrafficLight::RuleName, getId(num), map);
}

inline RegulatoryElementPtr setUpGenericRegulatoryElement(int& num) {
  return RegulatoryElementFactory::create(GenericRegulatoryElement::RuleName, getId(num), RuleParameterMap{});
}

inline Lanelet setUpLanelet(int& num, const std::string& type = AttributeValueString::Road) {
  auto regelem = setUpRegulatoryElement(num);
  return Lanelet(getId(num), setUpLineString(num), setUpLineString(num),
                 AttributeMap{{AttributeNamesString::Subtype, type}}, {regelem});
}

inline bool hasId(const Ids& ids, Id id) { return std::find(ids.begin(), ids.end(), id) != ids.end(); }

inline Area setUpArea(int& num, const std::string& type = AttributeValueString::Parking) {
  Point3d o1(getId(num), 0, 0, 0);
  Point3d o2(getId(num), 0, 5, 0);
  Point3d o3(getId(num), 5, 5, 0);
  Point3d o4(getId(num), 5, 0, 0);

  Point3d i11(getId(num), 1, 1, 0);
  Point3d i12(getId(num), 1, 2, 0);
  Point3d i13(getId(num), 2, 2, 0);
  Point3d i14(getId(num), 2, 1, 0);
  Point3d i21(getId(num), 3, 3, 0);
  Point3d i22(getId(num), 3, 4, 0);
  Point3d i23(getId(num), 4, 4, 0);

  LineString3d lsO1(getId(num), {o1, o2, o3});
  LineString3d lsO2(getId(num), {o1, o4, o3});
  LineString3d lsI11(getId(num), {i11, i12, i13});
  LineString3d lsI12(getId(num), {i11, i14, i13});
  LineString3d lsI21(getId(num), {i22, i21});
  LineString3d lsI22(getId(num), {i21, i23});
  LineString3d lsI23(getId(num), {i22, i23});
  auto regelem = setUpRegulatoryElement(num);
  return Area(getId(num), {lsO1, lsO2}, {{lsI11, lsI12}, {lsI21, lsI22, lsI23}},
              AttributeMap{{AttributeNamesString::Subtype, type}}, {regelem});
}

class Tempfile {
 public:
  explicit Tempfile(std::string name) {
    char dir[] = {"/tmp/lanelet2_io_test.XXXXXX"};
    auto* res = mkdtemp(dir);
    if (res == nullptr) {
      throw lanelet::LaneletError("Failed to crate temporary directory");
    }
    dir_ = dir;
    path_ = fs::path(dir_) / name;
  }
  Tempfile(Tempfile&& rhs) noexcept = delete;
  Tempfile& operator=(Tempfile&& rhs) noexcept = delete;
  Tempfile(const Tempfile& rhs) = delete;
  Tempfile& operator=(const Tempfile& rhs) = delete;
  ~Tempfile() { fs::remove_all(path_); }

  const fs::path& get() const { return path_; }
  void touch() { std::ofstream(path_.string()); }

 private:
  std::string dir_;
  fs::path path_;
};
}  // namespace test_setup
}  // namespace lanelet
