#include "lanelet2_ml_converter/Utils.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace fs = boost::filesystem;
namespace lanelet {
namespace ml_converter {

OrientedRect getRotatedRect(const BasicPoint3d& center, double extentLongitudinal, double extentLateral, double yaw,
                            bool from2dPos) {
  BasicPoints2d pts{BasicPoint2d{center.x() - extentLongitudinal, center.y() - extentLateral},
                    BasicPoint2d{center.x() - extentLongitudinal, center.y() + extentLateral},
                    BasicPoint2d{center.x() + extentLongitudinal, center.y() + extentLateral},
                    BasicPoint2d{center.x() + extentLongitudinal, center.y() - extentLateral},
                    BasicPoint2d{center.x() - extentLongitudinal, center.y() - extentLateral}};
  OrientedRect axisAlignedRect;
  boost::geometry::assign_points(axisAlignedRect.bg_poly, pts);
  boost::geometry::correct(axisAlignedRect.bg_poly);

  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> trans1(-center.x(), -center.y());
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(-yaw);
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> trans2(center.x(), center.y());

  OrientedRect trans1Rect;
  OrientedRect rotatedRect;
  OrientedRect trans2Rect;
  boost::geometry::transform(axisAlignedRect.bg_poly, trans1Rect.bg_poly, trans1);
  boost::geometry::transform(trans1Rect.bg_poly, rotatedRect.bg_poly, rotate);
  boost::geometry::transform(rotatedRect.bg_poly, trans2Rect.bg_poly, trans2);

  trans2Rect.center = center;
  trans2Rect.yaw = yaw;
  trans2Rect.from2d = from2dPos;
  return trans2Rect;
}

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center,
                                    double extentLongitudinal, double extentLateral) {
  double maxExtent = sqrt(extentLongitudinal * extentLongitudinal + extentLateral * extentLateral);
  BasicPoint2d initRegionRear = {center.x() - 1.1 * maxExtent, center.y() - 1.1 * maxExtent};
  BasicPoint2d initRegionFront = {center.x() + 1.1 * maxExtent, center.y() + 1.1 * maxExtent};
  BoundingBox2d initSearchRegion{initRegionRear, initRegionFront};
  ConstLanelets initRegion = laneletMap->laneletLayer.search(initSearchRegion);
  return utils::createConstSubmap(initRegion, {});
}

/// TODO: FURTHER INVESTIGATE THE WEIRD BEHAVIOR OF BOOST LINE_INTERPOLATE
BasicLineString3d resampleLineString(const BasicLineString3d& polyline, int32_t nPoints) {
  if (polyline.size() < 1) {
    throw std::runtime_error("A polyline requires at least 2 points!");
  }
  double length = boost::geometry::length(polyline, boost::geometry::strategy::distance::pythagoras<double>());
  if (length < 1e-1) {
    return BasicLineString3d();
  }
  double dist = length / static_cast<double>(nPoints - 1);  // to get all points of line
  boost::geometry::model::multi_point<BasicPoint3d> bdInterp;
  boost::geometry::line_interpolate(polyline, dist, bdInterp);
  bdInterp.insert(bdInterp.begin(), polyline.front());
  if (bdInterp.size() != nPoints && (bdInterp.back() - polyline.back()).norm() > 10e-5) {
    bdInterp.insert(bdInterp.end(), polyline.back());
  } else if (bdInterp.size() == nPoints && (bdInterp.back() - polyline.back()).norm() > 10e-5) {
    bdInterp.back() = polyline.back();
  } else if (bdInterp.size() != nPoints) {
    std::stringstream ss;
    for (const auto& pt : bdInterp) {
      ss << pt;
    }
    throw std::runtime_error("LineString resampling failed to produce the desired number of points: " + ss.str());
  }
  return bdInterp;
}

std::vector<BasicLineString3d> cutLineString(const OrientedRect& bbox, const BasicLineString3d& polyline) {
  BasicLineString2d polyline2d;
  for (const auto& pt : polyline) {
    polyline2d.push_back(BasicPoint2d(pt.x(), pt.y()));
  }
  std::deque<BasicLineString2d> cut2d;
  boost::geometry::intersection(bbox.bg_poly, polyline2d, cut2d);

  std::vector<BasicLineString3d> cut3d;
  if (cut2d.empty()) {
    return cut3d;
  } else if (cut2d.size() > 1) {
    // std::cerr << "More than one cut line!" << std::endl;
    // std::cerr << "Raw line:" << std::endl;
    // for (const auto& pt : polyline) {
    //   std::cerr << "[" << pt.x() << "," << pt.y() << "]" << std::endl;
    // }
    // for (const auto& ls : cut2d) {
    //   std::cerr << "Cut line:" << std::endl;
    //   for (const auto& pt : ls) {
    //     std::cerr << "[" << pt.x() << "," << pt.y() << "]" << std::endl;
    //   }
    //   std::cerr << "--------------" << std::endl;
    // }
    // throw std::runtime_error("More than one cut line!");
  }

  // restore z value from closest point on the original linestring
  for (const auto& el : cut2d) {
    BasicLineString3d ls;
    for (const auto& pt2d : el) {
      double lastDist = std::numeric_limits<double>::max();
      double bestZ;
      for (const auto& pt : polyline) {
        double currDist = (pt2d - BasicPoint2d(pt.x(), pt.y())).norm();
        if (currDist < lastDist) {
          lastDist = currDist;
          bestZ = pt.z();
        }
      }
      ls.push_back(BasicPoint3d(pt2d.x(), pt2d.y(), bestZ));
    }
    cut3d.push_back(ls);
  }
  return cut3d;
}

boost::geometry::strategy::transform::matrix_transformer<double, 3, 3> getYPRMatrix(double yaw, double pitch,
                                                                                    double roll) {
  Eigen::AngleAxisd rollAngle(-roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(-pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(-yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  Eigen::Matrix3d rotM = q.matrix();
  boost::geometry::strategy::transform::matrix_transformer<double, 3, 3> ypr(
      rotM(0, 0), rotM(0, 1), rotM(0, 2), 0, rotM(1, 0), rotM(1, 1), rotM(1, 2), 0, rotM(2, 0), rotM(2, 1), rotM(2, 2),
      0, 0, 0, 0, 1);
  return ypr;
}

BasicLineString3d transformLineString(const OrientedRect& bbox, const BasicLineString3d& polyline, double pitch,
                                      double roll) {
  boost::geometry::strategy::transform::translate_transformer<double, 3, 3> trans1(-bbox.center.x(), -bbox.center.y(),
                                                                                   -bbox.center.z());
  boost::geometry::strategy::transform::matrix_transformer<double, 3, 3> ypr = getYPRMatrix(bbox.yaw, pitch, roll);
  BasicLineString3d transPolyline;
  BasicLineString3d rotatedPolyline;
  boost::geometry::transform(polyline, transPolyline, trans1);
  boost::geometry::transform(transPolyline, rotatedPolyline, ypr);
  return rotatedPolyline;
}

void saveLaneData(const std::string& filename, const std::vector<LaneDataPtr>& lDataVec, bool binary) {
  if (binary) {
    std::ofstream fs(filename, std::ofstream::binary);
    if (!fs.good()) {
      throw std::runtime_error("Failed to open archive " + filename);
    }
    boost::archive::binary_oarchive oa(fs);
    oa << lDataVec;
  } else {
    std::ofstream fs(filename);
    if (!fs.good()) {
      throw std::runtime_error("Failed to open archive " + filename);
    }
    boost::archive::xml_oarchive oa(fs, boost::archive::no_header);
    oa << BOOST_SERIALIZATION_NVP(lDataVec);
  }
}

std::vector<LaneDataPtr> loadLaneData(const std::string& filename, bool binary) {
  if (!fs::exists(fs::path(filename))) {
    throw std::runtime_error("Could not find file under " + filename);
  }
  std::vector<LaneDataPtr> lDataVec;
  if (binary) {
    std::ifstream fs(filename, std::ifstream::binary);
    if (!fs.good()) {
      throw std::runtime_error("Failed to open archive " + filename);
    }
    boost::archive::binary_iarchive ia(fs);
    ia >> lDataVec;
  } else {
    std::ifstream fs(filename);
    if (!fs.good()) {
      throw std::runtime_error("Failed to open archive " + filename);
    }
    boost::archive::xml_iarchive ia(fs, boost::archive::no_header);
    ia >> BOOST_SERIALIZATION_NVP(lDataVec);
  }

  return lDataVec;
}

void saveLaneDataMultiFile(const std::string& path, const std::vector<std::string>& filenames,
                           const std::vector<LaneDataPtr>& lDataVec, bool binary) {
  if (filenames.size() != lDataVec.size()) {
    throw std::runtime_error("Unequal number of file names and LaneData objects!");
  }
  for (size_t i = 0; i < filenames.size(); i++) {
    const auto& filename = filenames[i];
    const auto& lData = lDataVec[i];
    if (binary) {
      std::ofstream fs(path + filename, std::ofstream::binary);
      if (!fs.good()) {
        throw std::runtime_error("Failed to open archive " + filename);
      }
      boost::archive::binary_oarchive oa(fs);
      oa << lDataVec;
    } else {
      std::ofstream fs(path + filename);
      if (!fs.good()) {
        throw std::runtime_error("Failed to open archive " + filename);
      }
      boost::archive::xml_oarchive oa(fs, boost::archive::no_header);
      oa << BOOST_SERIALIZATION_NVP(lData);
    }
  }
}

std::vector<LaneDataPtr> loadLaneDataMultiFile(const std::string& path, const std::vector<std::string>& filenames,
                                               bool binary) {
  std::vector<LaneDataPtr> lDataVec;
  for (size_t i = 0; i < filenames.size(); i++) {
    const auto& filename = filenames[i];
    LaneDataPtr lData;
    if (!fs::exists(fs::path(path + filename))) {
      throw std::runtime_error("Could not find file under " + filename);
    }
    if (binary) {
      std::ifstream fs(path + filename, std::ifstream::binary);
      if (!fs.good()) {
        throw std::runtime_error("Failed to open archive " + filename);
      }
      boost::archive::binary_iarchive ia(fs);
      ia >> lData;
    } else {
      std::ifstream fs(path + filename);
      if (!fs.good()) {
        throw std::runtime_error("Failed to open archive " + filename);
      }
      boost::archive::xml_iarchive ia(fs, boost::archive::no_header);
      ia >> BOOST_SERIALIZATION_NVP(lData);
    }
    lDataVec.push_back(lData);
  }
  return lDataVec;
}
}  // namespace ml_converter
}  // namespace lanelet