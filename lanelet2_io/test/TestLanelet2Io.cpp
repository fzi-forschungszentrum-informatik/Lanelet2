#include "gtest/gtest.h"

#include "Exceptions.h"
#include "Io.h"

TEST(lanelet2_io, registryTest) {  // NOLINT
  auto parseExtensions = lanelet::supportedParserExtensions();
  EXPECT_NE(std::find(parseExtensions.begin(), parseExtensions.end(), ".osm"), parseExtensions.end());
  EXPECT_NE(std::find(parseExtensions.begin(), parseExtensions.end(), ".bin"), parseExtensions.end());

  auto writeExtensions = lanelet::supportedWriterExtensions();
  EXPECT_NE(std::find(writeExtensions.begin(), writeExtensions.end(), ".osm"), writeExtensions.end());
  EXPECT_NE(std::find(writeExtensions.begin(), writeExtensions.end(), ".bin"), writeExtensions.end());
}

TEST(lanelet2_io, exceptionTest) {  // NOLINT
  auto nonsenseExtension = std::string(std::tmpnam(nullptr)) + ".unsupported_extension";
  std::FILE* tmpf = std::fopen(nonsenseExtension.c_str(), "wb+");
  EXPECT_THROW(lanelet::load(nonsenseExtension), lanelet::UnsupportedExtensionError);                        // NOLINT
  EXPECT_THROW(lanelet::load(nonsenseExtension, "nonexisting_parser"), lanelet::UnsupportedIOHandlerError);  // NOLINT
  EXPECT_THROW(lanelet::load("/nonexisting/file/with/known/extension.osm"), lanelet::FileNotFoundError);     // NOLINT
  std::fclose(tmpf);
  std::remove(nonsenseExtension.c_str());
}
