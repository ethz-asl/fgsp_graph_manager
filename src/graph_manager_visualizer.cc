#include "graph_manager/graph_manager_visualizer.h"

namespace fgsp {

GraphManagerVisualizer::GraphManagerVisualizer(GraphManagerConfig const& config, GraphManagerPublisher& publisher)
    : config_(config), publisher_(publisher) {}

}  // namespace fgsp
