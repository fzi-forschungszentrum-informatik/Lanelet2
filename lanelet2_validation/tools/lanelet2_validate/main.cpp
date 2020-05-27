#include "lanelet2_validation/Cli.h"

int main(int argc, char* argv[]) {
  auto config = lanelet::validation::parseCommandLine(argc, const_cast<const char**>(argv));  // NOLINT
  return lanelet::validation::runFromConfig(config);
}
