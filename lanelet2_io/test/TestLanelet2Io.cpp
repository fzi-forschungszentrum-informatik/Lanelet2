#include "TestSetup.h"
#include "gtest/gtest.h"
#include "lanelet2_io/Exceptions.h"
#include "lanelet2_io/Io.h"

TEST(lanelet2_io, registryTest) {  // NOLINT
  auto parseExtensions = lanelet::supportedParserExtensions();
  EXPECT_NE(std::find(parseExtensions.begin(), parseExtensions.end(), ".osm"), parseExtensions.end());
  EXPECT_NE(std::find(parseExtensions.begin(), parseExtensions.end(), ".bin"), parseExtensions.end());

  auto writeExtensions = lanelet::supportedWriterExtensions();
  EXPECT_NE(std::find(writeExtensions.begin(), writeExtensions.end(), ".osm"), writeExtensions.end());
  EXPECT_NE(std::find(writeExtensions.begin(), writeExtensions.end(), ".bin"), writeExtensions.end());
}

TEST(lanelet2_io, exceptionTest) {  // NOLINT
  lanelet::test_setup::Tempfile file("file_with.unsupported_extension");
  file.touch();
  EXPECT_THROW(lanelet::load(file.get().string()), lanelet::UnsupportedExtensionError);                        // NOLINT
  EXPECT_THROW(lanelet::load(file.get().string(), "nonexisting_parser"), lanelet::UnsupportedIOHandlerError);  // NOLINT
  EXPECT_THROW(lanelet::load("/nonexisting/file/with/known/extension.osm"), lanelet::FileNotFoundError);       // NOLINT
}
