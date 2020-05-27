#pragma once
#include "lanelet2_validation/Validation.h"

namespace lanelet {
namespace validation {
struct CommandLineConfig {
  ValidationConfig validationConfig;
  std::string mapFile;
  bool print{false};
  bool help{false};
};

//! obtain the configuration from command line arguments
CommandLineConfig parseCommandLine(int argc, const char* argv[]);

//! prints a vector of issues to the command line
void printAllIssues(const std::vector<DetectedIssues>& issues);

//! Runs the configuration and returns the programs's return value (0 on success, 1 if issues found)
int runFromConfig(const CommandLineConfig& config);

}  // namespace validation
}  // namespace lanelet
