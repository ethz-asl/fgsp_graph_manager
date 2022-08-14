#include "graph_manager/graph_manager_logger.h"

namespace fgsp {

std::unique_ptr<GraphManagerLogger> GraphManagerLogger::instance_ = nullptr;

auto GraphManagerLogger::getInstance() -> GraphManagerLogger const& {
  if (instance_ == nullptr) {
    instance_.reset(new GraphManagerLogger());
  }
  return *instance_;
}

void GraphManagerLogger::logInfo(const std::string& msg) const {
  std::cout << "[INFO] " << msg << "\n";
}

void GraphManagerLogger::logWarn(const std::string& msg) const {
  std::cout << "\033[33m[WARNING] " << msg << "\033[0m\n";
}

void GraphManagerLogger::logError(const std::string& msg) const {
  std::cerr << "\033[31m[ERROR] " << msg << "\033[0m\n";
}

}  // namespace fgsp
