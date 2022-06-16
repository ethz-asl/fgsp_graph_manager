#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace fgsp {

class GraphManagerPublisher {
 public:
  GraphManagerPublisher(rclcpp::Node& node);

  template <typename T>
  void publish(const T& msg, const std::string& topic) {
    auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(publishers_[topic]);
    if (!publisher) {
      std::cout << "creating publisher for topic " << topic << std::endl;
      publisher = node_.create_publisher<T>(topic, 10);
      publishers_[topic] = std::dynamic_pointer_cast<rclcpp::PublisherBase>(publisher);
    }
    std::cout << "Publishging at topic " << topic << std::endl;
    publisher->publish(msg);
  }

 private:
  rclcpp::Node& node_;
  std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> publishers_;
};

}  // namespace fgsp
