#include "graph_manager/graph_manager_node.h"

GraphManagerNode::GraphManagerNode()
    : rclcpp::Node("graph_manager") {
  std::cout << "Initializing Node..." << std::endl;

  config_.reset(fgsp::GraphManagerConfig::init(*this));
  manager_ = std::make_unique<fgsp::GraphManager>(*config_);

  // if (maplabIntegrator_->setup(nh_, pnh_))
  // std::cout << "--- Laser Maplab Integrator Nodelet Initialized ---" << std::endl;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto graph_manager_node = std::make_shared<GraphManagerNode>();

  rclcpp::spin(graph_manager_node);

  rclcpp::shutdown();
  return 0;
}