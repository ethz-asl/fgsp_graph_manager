#include "graph_manager/graph_manager_logger.h"
#include <iomanip>

namespace fgsp {

std::unique_ptr<GraphManagerLogger> GraphManagerLogger::instance_ = nullptr;

auto GraphManagerLogger::getInstance() -> GraphManagerLogger const& {
  if (instance_ == nullptr) {
    instance_.reset(new GraphManagerLogger());
  }
  return *instance_;
}

void GraphManagerLogger::logInfo(const std::string& msg) const {
  std::cout << std::fixed << std::setprecision(3) << "[INFO] " << msg << "\n";
}

void GraphManagerLogger::logWarn(const std::string& msg) const {
  std::cout << std::fixed << std::setprecision(3) << "\033[33m[WARNING] " << msg
            << "\033[0m\n";
}

void GraphManagerLogger::logError(const std::string& msg) const {
  std::cerr << std::fixed << std::setprecision(3) << "\033[31m[ERROR] " << msg
            << "\033[0m\n";
}

}  // namespace fgsp
