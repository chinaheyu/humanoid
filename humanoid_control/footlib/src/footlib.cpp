#include "footlib.h"
#include "foot_forward.h"
#include "foot_jacobian.h"


namespace humanoid {

bool FootLib::forward(const Eigen::Matrix<double, 3, 2>& upper_lower, Eigen::Matrix<double, 3, 2>& roll_pitch, double tolerance, int max_iter) {
    Eigen::Vector2d upper_lower_theta = upper_lower.row(0);

    // Jacobian inverse method to solve forward kinematics.
    Eigen::Vector2d temp = Eigen::Vector2d::Zero();
    Eigen::Matrix2d invJ;
    Eigen::Matrix2d J;
    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector2d yhat;
        
        Eigen::Vector2d err;
        Eigen::Vector2d delta;
        double total_err;

        foot_forward(roll_pitch(0, 0), roll_pitch(0, 1), upper_lower_theta.data());

        err = upper_lower_theta - yhat;
        total_err = err.cwiseAbs().sum();

        if (total_err < tolerance)
            break;

        J = jacobian(temp);
        invJ = J.completeOrthogonalDecomposition().pseudoInverse();

        delta = invJ * err;
        temp += delta;
    }
    if (temp(0) < -1.0 || temp(0) > 1.0 || temp(1) < -1.57 || temp(1) > 1.57) {
        return false;
    }
    roll_pitch.row(0) = temp;

    // Calculate velocity and torque.
    J = jacobian(temp);
    invJ = J.completeOrthogonalDecomposition().pseudoInverse();
    roll_pitch.row(1) = invJ * upper_lower.row(1).transpose();
    roll_pitch.row(2) = J.transpose().completeOrthogonalDecomposition().pseudoInverse() * upper_lower.row(2).transpose();

    return true;
}

void FootLib::backward(const Eigen::Matrix<double, 3, 2>& roll_pitch, Eigen::Matrix<double, 3, 2>& upper_lower_theta) {
    Eigen::Vector2d theta;
    foot_forward(roll_pitch(0), roll_pitch(1), theta.data());
    upper_lower_theta.row(0) = theta;

    Eigen::Matrix2d J = jacobian(roll_pitch.row(0));
    upper_lower_theta.row(1) = J * roll_pitch.row(1).transpose();
    upper_lower_theta.row(2) = J.transpose() * roll_pitch.row(2).transpose();
}

Eigen::Matrix2d FootLib::jacobian(Eigen::Vector2d roll_pitch) {
    Eigen::Matrix2d j;
    foot_jacobian(roll_pitch(0), roll_pitch(1), j.data());
    return j;
}

}
