#pragma once

#include <iostream>
#include <memory>

namespace fgsp {

class GraphManagerLogger {
 private:
  GraphManagerLogger() = default;
  static std::unique_ptr<GraphManagerLogger> instance_;

 public:
  static auto getInstance() -> GraphManagerLogger const&;
  void logInfo(const std::string& msg) const;
  void logError(const std::string& msg) const;
};

}  // namespace fgsp