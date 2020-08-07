#include "lanelet2_io/Io.h"

#include <boost/filesystem.hpp>

#include "lanelet2_io/Exceptions.h"
#include "lanelet2_io/io_handlers/Factory.h"

namespace fs = boost::filesystem;

namespace lanelet {
namespace {
std::string extension(const std::string& path) { return fs::path(path).extension().string(); }

template <typename ExceptionT>
void handleErrorsOrThrow(const ErrorMessages& errors, ErrorMessages* targetErrs) {
  if (targetErrs != nullptr) {
    *targetErrs = errors;
  } else if (!errors.empty()) {
    throw ExceptionT(errors);
  }
}
}  // namespace

std::unique_ptr<LaneletMap> load(const std::string& filename, const Origin& origin, ErrorMessages* errors,
                                 const io::Configuration& params) {
  return load(filename, defaultProjection(origin), errors, params);
}

std::unique_ptr<LaneletMap> load(const std::string& filename, const Projector& projector, ErrorMessages* errors,
                                 const io::Configuration& params) {
  if (!fs::exists(fs::path(filename))) {
    throw FileNotFoundError("Could not find lanelet map under " + filename);
  }
  ErrorMessages err;
  auto map =
      io_handlers::ParserFactory::createFromExtension(extension(filename), projector, params)->parse(filename, err);
  handleErrorsOrThrow<ParseError>(err, errors);
  return map;
}

std::unique_ptr<LaneletMap> load(const std::string& filename, const std::string& parserName, const Origin& origin,
                                 ErrorMessages* errors, const io::Configuration& params) {
  return load(filename, parserName, defaultProjection(origin), errors, params);
}

std::unique_ptr<LaneletMap> load(const std::string& filename, const std::string& parserName, const Projector& projector,
                                 ErrorMessages* errors, const io::Configuration& params) {
  if (!fs::exists(fs::path(filename))) {
    throw FileNotFoundError("Could not find lanelet map under " + filename);
  }
  ErrorMessages err;
  auto map = io_handlers::ParserFactory::create(parserName, projector, params)->parse(filename, err);
  handleErrorsOrThrow<ParseError>(err, errors);
  return map;
}

std::vector<std::string> supportedParsers() { return io_handlers::ParserFactory::availableParsers(); }

std::vector<std::string> supportedParserExtensions() { return io_handlers::ParserFactory::availableExtensions(); }

void write(const std::string& filename, const LaneletMap& map, const Origin& origin, ErrorMessages* errors,
           const io::Configuration& params) {
  write(filename, map, defaultProjection(origin), errors, params);
}

void write(const std::string& filename, const LaneletMap& map, const Projector& projector, ErrorMessages* errors,
           const io::Configuration& params) {
  ErrorMessages err;
  io_handlers::WriterFactory::createFromExtension(extension(filename), projector, params)->write(filename, map, err);
  handleErrorsOrThrow<WriteError>(err, errors);
}

void write(const std::string& filename, const LaneletMap& map, const std::string& writerName, const Origin& origin,
           ErrorMessages* errors, const io::Configuration& params) {
  write(filename, map, writerName, defaultProjection(origin), errors, params);
}

void write(const std::string& filename, const LaneletMap& map, const std::string& writerName,
           const Projector& projector, ErrorMessages* errors, const io::Configuration& params) {
  ErrorMessages err;
  io_handlers::WriterFactory::create(writerName, projector, params)->write(filename, map, err);
  handleErrorsOrThrow<WriteError>(err, errors);
}

std::vector<std::string> supportedWriters() { return io_handlers::WriterFactory::availableWriters(); }

std::vector<std::string> supportedWriterExtensions() { return io_handlers::WriterFactory::availableExtensions(); }
}  // namespace lanelet
