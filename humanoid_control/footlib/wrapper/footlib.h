#ifndef __FOOTLIB_H__
#define __FOOTLIB_H__

#include <eigen3/Eigen/Eigen>

namespace humanoid {
    struct FootLib {
        bool forward(const Eigen::Vector2d& upper_lower_theta, Eigen::Vector2d& roll_pitch, double tolerance=0.001, int max_iter=10);
        void backward(const Eigen::Vector2d& roll_pitch, Eigen::Vector2d& upper_lower_theta);
        Eigen::Matrix2d jacobian(Eigen::Vector2d roll_pitch);
    };
}

#endif // __FOOTLIB_H__
