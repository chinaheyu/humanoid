#ifndef __HUMANOID_CONTROL_NODE_H__
#define __HUMANOID_CONTROL_NODE_H__

#include <rclcpp/rclcpp.hpp>

#include "humanoid_interface/visibility_control.h"

namespace humanoid {

class HumanoidControlNode : public rclcpp::Node {
public:
    COMPOSITION_PUBLIC
    explicit HumanoidControlNode(const rclcpp::NodeOptions& options);
};

}  // namespace humanoid

#endif  // __HUMANOID_CONTROL_NODE_H__
