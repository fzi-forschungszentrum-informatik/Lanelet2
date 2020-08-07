#include "lanelet2_io/io_handlers/Factory.h"

#include "lanelet2_io/Exceptions.h"

namespace lanelet {
namespace io_handlers {
namespace {
std::string format(const std::vector<std::string>& strings, const std::string& delim = ", ") {
  std::string formatted;
  for (const auto& str : strings) {
    if (!formatted.empty()) {
      formatted += delim;
    }
    formatted += str;
  }
  return formatted;
}
}  // namespace

//===============================================================================================================
//              CLASS PARSER FACTORY
//===============================================================================================================
Parser::Ptr ParserFactory::create(const std::string& parserName, const Projector& projector,
                                  const io::Configuration& config) {
  ParserFactory& inst = ParserFactory::instance();
  auto it = inst.registry_.find(parserName);
  if (it != inst.registry_.end()) {
    auto newObj = std::shared_ptr<Parser>(it->second(projector, config));
    return newObj;
  }
  throw UnsupportedIOHandlerError("Requested parser " + parserName +
                                  " does not exist! Available parsers are: " + format(availableParsers()));
}

Parser::Ptr ParserFactory::createFromExtension(const std::string& extension, const Projector& projector,
                                               const io::Configuration& config) {
  ParserFactory& inst = ParserFactory::instance();
  auto it = inst.extensionRegistry_.find(extension);
  if (it != inst.extensionRegistry_.end()) {
    auto newObj = std::shared_ptr<Parser>(it->second(projector, config));
    return newObj;
  }
  throw UnsupportedExtensionError("Requested extension " + extension +
                                  " is not supported! Supported extensions are: " + format(availableExtensions()));
}

std::vector<std::string> ParserFactory::availableParsers() {
  std::vector<std::string> parsers;
  for (const auto& parser : ParserFactory::instance().registry_) {
    parsers.push_back(parser.first);
  }
  std::sort(parsers.begin(), parsers.end());
  return parsers;
}

std::vector<std::string> ParserFactory::availableExtensions() {
  std::vector<std::string> extensions;
  for (const auto& parser : ParserFactory::instance().extensionRegistry_) {
    extensions.push_back(parser.first);
  }
  std::sort(extensions.begin(), extensions.end());
  return extensions;
}

ParserFactory& ParserFactory::instance() {
  static ParserFactory factory;
  return factory;
}

void ParserFactory::registerParser(const std::string& strategy, const std::string& extension,
                                   const ParserCreationFcn& factoryFunction) {
  registry_[strategy] = factoryFunction;
  if (!extension.empty()) {
    extensionRegistry_[extension] = factoryFunction;
  }
}

//===============================================================================================================
//              CLASS WRITER FACTORY
//===============================================================================================================
Writer::Ptr WriterFactory::create(const std::string& writerName, const lanelet::Projector& projector,
                                  const io::Configuration& config) {
  WriterFactory& inst = WriterFactory::instance();
  auto it = inst.registry_.find(writerName);
  if (it != inst.registry_.end()) {
    auto newObj = std::shared_ptr<Writer>(it->second(projector, config));
    return newObj;
  }
  throw UnsupportedIOHandlerError("Requested writer " + writerName +
                                  " does not exist! Available writer are: " + format(availableWriters()));
}

Writer::Ptr WriterFactory::createFromExtension(const std::string& extension, const Projector& projector,
                                               const io::Configuration& config) {
  WriterFactory& inst = WriterFactory::instance();
  auto it = inst.extensionRegistry_.find(extension);
  if (it != inst.extensionRegistry_.end()) {
    auto newObj = std::shared_ptr<Writer>(it->second(projector, config));
    return newObj;
  }
  throw UnsupportedExtensionError("Requested extension " + extension +
                                  " is not supported! Supported extensions are: " + format(availableExtensions()));
}

std::vector<std::string> WriterFactory::availableWriters() {
  std::vector<std::string> writers;
  for (const auto& writer : WriterFactory::instance().registry_) {
    writers.push_back(writer.first);
  }
  std::sort(writers.begin(), writers.end());
  return writers;
}

std::vector<std::string> WriterFactory::availableExtensions() {
  std::vector<std::string> extensions;
  for (const auto& writer : WriterFactory::instance().extensionRegistry_) {
    extensions.push_back(writer.first);
  }
  std::sort(extensions.begin(), extensions.end());
  return extensions;
}

WriterFactory& WriterFactory::instance() {
  static WriterFactory factory;
  return factory;
}

void WriterFactory::registerWriter(const std::string& strategy, const std::string& extension,
                                   const WriterCreationFcn& factoryFunction) {
  registry_[strategy] = factoryFunction;
  if (!extension.empty()) {
    extensionRegistry_[extension] = factoryFunction;
  }
}
}  // namespace io_handlers
}  // namespace lanelet
