#include "io_handlers/BinHandler.h"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>
#include "Exceptions.h"
#include "io_handlers/Factory.h"
#include "io_handlers/Serialize.h"

namespace lanelet {
namespace io_handlers {

namespace {
// register with factories
static RegisterParser<BinParser> regParser;
static RegisterWriter<BinWriter> regWriter;

}  // namespace

void BinWriter::write(const std::string& filename, const LaneletMap& laneletMap, ErrorMessages& /*errors*/) const {
  std::ofstream fs(filename, std::ofstream::binary);
  if (!fs.good()) {
    throw ParseError("Failed open archive " + filename);
  }
  boost::archive::binary_oarchive oa(fs);
  oa << laneletMap;
}

std::unique_ptr<LaneletMap> BinParser::parse(const std::string& filename, ErrorMessages& /*errors*/) const {
  std::ifstream fs(filename, std::ifstream::binary);
  if (!fs.good()) {
    throw ParseError("Failed open archive " + filename);
  }
  std::unique_ptr<LaneletMap> laneletMap = std::make_unique<LaneletMap>();
  boost::archive::binary_iarchive ia(fs);
  ia >> *laneletMap;
  return laneletMap;
}
}  // namespace io_handlers
}  // namespace lanelet
