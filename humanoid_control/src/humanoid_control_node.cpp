#include "humanoid_control/humanoid_control_node.h"

namespace humanoid {

HumanoidControlNode::HumanoidControlNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("humanoid_base", options), state_estimator_(this) {}

}  // namespace humanoid

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(humanoid::HumanoidControlNode)