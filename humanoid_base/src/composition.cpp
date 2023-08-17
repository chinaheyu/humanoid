#include <memory>

#include "humanoid_base/humanoid_base_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<humanoid::HumanoidBaseNode>(
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}