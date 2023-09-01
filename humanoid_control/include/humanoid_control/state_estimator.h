#ifndef __STATE_ESTIMATOR_H__
#define __STATE_ESTIMATOR_H__

#include <rclcpp/rclcpp.hpp>

namespace humanoid {

class StateEstimator {
public:
    explicit StateEstimator(rclcpp::Node* node);

private:
    rclcpp::Node* node_;
};

}  // namespace humanoid

#endif  // __STATE_ESTIMATOR_H__
