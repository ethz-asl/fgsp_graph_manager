#pragma once

#include <rclcpp/rclcpp.hpp>

#include "graph_manager/graph_manager_config.h"
#include "graph_manager/graph_manager_publisher.h"

namespace fgsp {

class GraphManagerVisualizer {
 public:
  explicit GraphManagerVisualizer(GraphManagerConfig const& config, GraphManagerPublisher& publisher);

 private:
  GraphManagerConfig const& config_;
  GraphManagerPublisher& publisher_;
};

}  // namespace fgsp