#include "graph_manager/graph_manager_publisher.h"

namespace fgsp {

GraphManagerPublisher::GraphManagerPublisher(rclcpp::Node& node)
    : node_(node) {}

rclcpp::Time GraphManagerPublisher::getTimeNow() const {
  return node_.get_clock()->now();
}

}  // namespace fgsp
