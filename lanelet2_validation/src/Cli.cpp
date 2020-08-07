#include "lanelet2_validation/Cli.h"

#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

namespace lanelet {
namespace validation {

CommandLineConfig parseCommandLine(int argc, const char* argv[]) {
  CommandLineConfig cfg;
  auto& config = cfg.validationConfig;
  po::options_description desc(
      "Runs a set of validators on a map. Think of it like a linter. The following checks are available:");
  desc.add_options()("help,h", "this help message")

      ("map_file", po::value<std::string>(), "Path to the map to be validated")

          ("filter,f", po::value(&config.checksFilter),
           "Comma separated list of regexes to filter the applicable tests. Will run all tests by default. Example: "
           "routing_graph.* to run all checks for the routing graph")

              ("location,l", po::value(&config.location)->default_value(config.location),
               "Location of the map (for instanciating the traffic rules), e.g. de for Germany")

                  ("participants,p", po::value(&config.participants)->composing(),
                   "Participants for which the routing graph will be instanciated (default: vehicle)")

                      ("lat", po::value(&config.origin.lat)->default_value(config.origin.lat),
                       "latitude coordinate of map origin")

                          ("lon", po::value(&config.origin.lon)->default_value(config.origin.lon),
                           "longitude coofdinate of map origin")

                              ("print", "Only print the checks that will be run, but dont run them");
  po::variables_map vm;
  po::positional_options_description pos;
  pos.add("map_file", 1);
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);
  cfg.help = vm.count("help") != 0;
  cfg.print = vm.count("print") != 0;
  if (vm.count("map_file") != 0) {
    cfg.mapFile = vm["map_file"].as<decltype(cfg.mapFile)>();
  }
  if (cfg.help) {
    std::cout << '\n' << desc;
  } else if (cfg.mapFile.empty() && !cfg.print) {
    std::cout << "Please pass either a valid file or '--print' or '--help'!\n";
  }
  return cfg;
}

void printAllIssues(const std::vector<DetectedIssues>& issues) {
  auto allIssues = buildReport(issues);
  for (auto& issue : allIssues.errors) {
    std::cerr << issue << '\n';
  }
  for (auto& issue : allIssues.warnings) {
    std::cout << issue << '\n';
  }
  std::cout << allIssues.warnings.size() + allIssues.errors.size() << " issues found.\n";
}

int runFromConfig(const CommandLineConfig& config) {
  if (config.help) {
    return 0;
  }
  if (config.print) {
    auto checks = availabeChecks(config.validationConfig.checksFilter);
    if (checks.empty()) {
      std::cout << "No checks found matching '" << config.validationConfig.checksFilter << "'\n";
    } else {
      std::cout << "Will use following checks:\n";
      for (auto& check : checks) {
        std::cout << check << '\n';
      }
    }
    return 0;
  }
  if (config.mapFile.empty()) {
    return 1;
  }
  auto issues = validateMap(config.mapFile, config.validationConfig);
  printAllIssues(issues);
  return int(!issues.empty());
}

}  // namespace validation
}  // namespace lanelet
