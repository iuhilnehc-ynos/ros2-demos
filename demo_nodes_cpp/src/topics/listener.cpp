// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        static bool once = true;
        static int i = 0;
        if (once) {
          std::string key = "Hello World: ";
          std::size_t pos = msg->data.find(key);
          i = std::stoi(msg->data.substr (pos + key.length()));
          // make the number as multiples of 5 or 10
          i = ((i / 5) + 1) * 5;
          once = false;
        }

        if (i < 100) {
          // To update with a new filter expression
          std::string filter_expression = "data MATCH %0";
          std::string expression_parameter = "'Hello World: " + std::to_string(i) + "'";

          RCLCPP_INFO(this->get_logger(), "set_cft_expression_parameters : [%s][%s]",
            filter_expression.c_str(), expression_parameter.c_str());
          sub_->set_cft_expression_parameters(filter_expression, {expression_parameter});
          i += 5;

          // To get filter expression
          try {
            std::string filter_expression_tmp;
            std::vector<std::string> expression_parameters;
            sub_->get_cft_expression_parameters(filter_expression_tmp, expression_parameters);
            RCLCPP_INFO(this->get_logger(), "get_cft_expression_parameters filter_expression: [%s]", filter_expression_tmp.c_str());
            for(auto &expression_parameter: expression_parameters) {
              RCLCPP_INFO(this->get_logger(), "get_cft_expression_parameters expression_parameter: [%s]", expression_parameter.c_str());
            }
          } catch (const std::exception& e) {
            RCLCPP_INFO(this->get_logger(), "get_cft_expression_parameters exception: %s", e.what());
          }
        } else {
          // reset
          RCLCPP_INFO(this->get_logger(), "reset cft");
          sub_->set_cft_expression_parameters("", {});
        }
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.

    rclcpp::SubscriptionOptions subscription_options;
    // subscription_options.content_filter_options.filter_expression
    //   = "data MATCH 'Hello World: 5' or data MATCH %0";
    // subscription_options.content_filter_options.expression_parameters = {"'Hello World: 10'"};
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback, subscription_options);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Listener)
