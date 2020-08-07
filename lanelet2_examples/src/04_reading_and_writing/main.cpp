#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

#include <cstdio>

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

// we want assert statements to work in release mode
#undef NDEBUG

namespace {
std::string exampleMapPath = std::string(PKG_DIR) + "/../lanelet2_maps/res/mapping_example.osm";

std::string tempfile(const std::string& name) {
  char tmpDir[] = "/tmp/lanelet2_example_XXXXXX";
  auto* file = mkdtemp(tmpDir);
  if (file == nullptr) {
    throw lanelet::IOError("Failed to open a temporary file for writing");
  }
  return std::string(file) + '/' + name;
}
}  // namespace

void part1LoadingAndWriting();
void part2Projectors();
void part3AddingNewParsersAndWriters();

int main() {
  // this tutorial shows you how to load and write lanelet maps. It is divided into three parts:
  part1LoadingAndWriting();
  part2Projectors();
  part3AddingNewParsersAndWriters();
  return 0;
}

void part1LoadingAndWriting() {
  using namespace lanelet;
  // loading a map requires two things: the path and either an origin or a projector that does the lat/lon->x/y
  // conversion.
  projection::UtmProjector projector(Origin({49, 8.4}));  // we will go into details later
  LaneletMapPtr map = load(exampleMapPath, projector);

  // the load and write functions are agnostic to the file extension. Depending on the extension, a different loading
  // algorithm will be chosen. Here we chose osm.

  // we can also load and write into an internal binary format. It is not human readable but loading is much faster
  // than from .osm:
  write(tempfile("map.bin"), *map);  // we do not need a projector to write to bin

  // if the map could not be parsed, exceptoins are thrown. Alternatively, you can provide an error struct. Then
  // lanelet2 will load the map as far as possible and write all the errors that occured to the error object that you
  // passed:
  ErrorMessages errors;
  map = load(exampleMapPath, projector, &errors);
  assert(errors.empty());  // of no errors occurred, the map could be fully parsed.
}

void part2Projectors() {
  using namespace lanelet;
  // as mentioned, projectors do the lat/lon->x/y conversion. This conversion is not trivial and only works when an
  // origin close to the actual map position is chosen. Otherwise the loaded map will be distorted.
  projection::UtmProjector projector(Origin({49, 8.4}));
  BasicPoint3d projection = projector.forward(GPSPoint{49, 8.4, 0});
  assert(std::abs(projection.x()) < 1e-6);

  // by default, lanele2 picks a projector that implements the mercator projection. However this is only due to legacy
  // reasons and because it is cheap and efficient to implement. In general, we recommend to use the UTM projector (we
  // already used it above). if you load a map from osm without providing a suitable projector or origin, an exception
  // will be thrown.
  // LaneletMapPtr map = load(exampleMapPath); // throws: loading from osm without projector
}

// you can easily add new parsers and writers so that they will be picked by load/write.
// here we write a writer that simply does nothing. We will not write a reader, but it works similarly.
namespace example {
class FakeWriter : public lanelet::io_handlers::Writer {
 public:
  using Writer::Writer;
  void write(const std::string& /*filename*/, const lanelet::LaneletMap& /*laneletMap*/,
             lanelet::ErrorMessages& /*errors*/) const override {
    // this writer does just nothing
  }
  static constexpr const char* extension() {
    return ".fake";  // this is the extension that we support
  }

  static constexpr const char* name() {
    return "fake_writer";  // this is the name of the writer. Users can also pick the writer by its name.
  }
};
}  // namespace example

namespace {
// this registers our new class for lanelet2_io
lanelet::io_handlers::RegisterWriter<example::FakeWriter> reg;
}  // namespace

void part3AddingNewParsersAndWriters() {
  using namespace lanelet;
  // now we can test our writer:
  LaneletMap map;
  write("anypath.fake", map);
}
