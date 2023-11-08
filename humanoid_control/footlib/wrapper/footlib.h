#ifndef __FOOTLIB_H__
#define __FOOTLIB_H__

#include <eigen3/Eigen/Eigen>

namespace humanoid {
    struct FootLib {
        static bool forward(const Eigen::Matrix<double, 3, 2>& upper_lower, Eigen::Matrix<double, 3, 2>& roll_pitch, double tolerance=0.001, int max_iter=10);
        static void backward(const Eigen::Matrix<double, 3, 2>& roll_pitch, Eigen::Matrix<double, 3, 2>& upper_lower_theta);
        static Eigen::Matrix2d jacobian(Eigen::Vector2d roll_pitch);
    };
}

#endif // __FOOTLIB_H__
