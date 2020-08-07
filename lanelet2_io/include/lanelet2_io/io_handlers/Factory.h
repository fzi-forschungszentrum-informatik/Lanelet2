#pragma once
#include <lanelet2_core/utility/Utilities.h>

#include <cstring>
#include <functional>
#include <map>
#include <string>

#include "lanelet2_io/Configuration.h"
#include "lanelet2_io/Projection.h"
#include "lanelet2_io/io_handlers/Parser.h"
#include "lanelet2_io/io_handlers/Writer.h"

namespace lanelet {
namespace io_handlers {
/**
 * @brief Factory class for all suppored lanelet map parsers
 */
class ParserFactory {  // NOLINT
 public:
  using ParserCreationFcn = std::function<Parser*(const Projector&, const io::Configuration&)>;

  static ParserFactory& instance();

  /**
   * @brief creates a parser that matches the given name.
   * @param parserName name of the parser (one of availableParsers())
   * @param projector projection object passed to the parser
   * @param config config object passed to the parser
   * @return created parser
   * @throws a lanelet2::UnsupportedIOHandlerError if handler is not registered
   */
  static Parser::Ptr create(const std::string& parserName, const Projector& projector,
                            const io::Configuration& config = io::Configuration());

  /**
   * @brief creates a matching parser for the given file extension
   * @param extension extension to look for (including the dot!)
   * @param projector projection object passed to the parser
   * @param config config object passed to the parser
   * @return created parser
   * @throws a lanelet2::UnsupportedExtensionError if extension is not
   * registered
   */
  static Parser::Ptr createFromExtension(const std::string& extension, const Projector& projector,
                                         const io::Configuration& config = io::Configuration());

  /**
   * @brief returns all available parsers as vector
   * @return vector of parser names
   */
  static std::vector<std::string> availableParsers();

  /**
   * @brief returns all available extensions as vector
   * @return vector of extensions (including the dot)
   */
  static std::vector<std::string> availableExtensions();

  template <typename T>
  friend class RegisterParser;

 private:
  void registerParser(const std::string& strategy, const std::string& extension,
                      const ParserCreationFcn& factoryFunction);

  ParserFactory() = default;
  std::map<std::string, ParserCreationFcn> registry_;
  std::map<std::string, ParserCreationFcn> extensionRegistry_;
};

/**
 * @brief Factory class for all supported lanelet map writers
 */
class WriterFactory {  // NOLINT
 public:
  using WriterCreationFcn = std::function<Writer*(const Projector&, const io::Configuration&)>;

  static WriterFactory& instance();

  /**
   * @brief creates a writer that matches the given name.
   * @param writerName name of the writer (one of availableParsers())
   * @param projector projection object passed to the writer
   * @param config config object passed to the writer
   * @return created writer
   * @throws a lanelet2::UnsupportedIOHandlerError if handler is not registered
   */
  static Writer::Ptr create(const std::string& writerName, const Projector& projector,
                            const io::Configuration& config = io::Configuration());

  /**
   * @brief creates a matching writer for the given file extension
   * @param extension extension to look for (including the dot!)
   * @param projector projection object passed to the writer
   * @param config config object passed to the writer
   * @return created writer
   * @throws a lanelet2::UnsupportedExtensionError if extension is not
   * registered
   */
  static Writer::Ptr createFromExtension(const std::string& extension, const Projector& projector,
                                         const io::Configuration& config = io::Configuration());

  /**
   * @brief returns all available writers as vector
   * @return vector of writer names
   */
  static std::vector<std::string> availableWriters();

  /**
   * @brief returns all available extensions as vector
   * @return vector of extensions (including the dot)
   */
  static std::vector<std::string> availableExtensions();

  template <typename T>
  friend class RegisterWriter;

 private:
  void registerWriter(const std::string& strategy, const std::string& extension,
                      const WriterCreationFcn& factoryFunction);

  WriterFactory() = default;
  std::map<std::string, WriterCreationFcn> registry_;
  std::map<std::string, WriterCreationFcn> extensionRegistry_;
};

/**
 * @brief Registration object for a writer. Needs to be instanciated as static
 * object once to register a writer.
 * Registration might look like this:
 *   static RegisterWriter<Mywriter> register;
 */
template <class T>
class RegisterWriter {
 public:
  RegisterWriter() {
    static_assert(!utils::strequal(T::name(), ""), "You did not overload the name() function!");
    WriterFactory::instance().registerWriter(
        T::name(), T::extension(), [](const Projector& projector, const io::Configuration& config) -> Writer* {
          return new T(projector, config);
        });
  }
};

/**
 * @brief Registration object for a parser. Needs to be instanciated as static
 * object once to register a parser.
 * Registration might look like this:
 *   static RegisterParser<Myparser> register;
 */
template <class T>
class RegisterParser {
 public:
  RegisterParser() {
    static_assert(!utils::strequal(T::name(), ""), "You did not overload the name() function!");
    ParserFactory::instance().registerParser(
        T::name(), T::extension(), [](const Projector& projector, const io::Configuration& config) -> Parser* {
          return new T(projector, config);
        });
  }
};
}  // namespace io_handlers
}  // namespace lanelet
