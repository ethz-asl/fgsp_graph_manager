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

void GraphManagerLogger::logError(const std::string& msg) const {
  std::cerr << "[ERROR] " << msg << "\n";
}

}  // namespace fgsp
