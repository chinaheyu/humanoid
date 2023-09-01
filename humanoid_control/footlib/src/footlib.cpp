#include "footlib.h"
#include "foot_forward.h"
#include "foot_jacobian.h"


namespace humanoid {

bool FootLib::forward(const Eigen::Vector2d& upper_lower_theta, Eigen::Vector2d& roll_pitch, double tolerance, int max_iter) {
    Eigen::Vector2d temp = Eigen::Vector2d::Zero();
    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector2d yhat;
        Eigen::Matrix2d invJ;
        Eigen::Vector2d err;
        Eigen::Vector2d delta;
        double total_err;

        backward(temp, yhat);

        err = upper_lower_theta - yhat;
        total_err = err.cwiseAbs().sum();

        if (total_err < tolerance)
            break;

        invJ = jacobian(temp).completeOrthogonalDecomposition().pseudoInverse();

        delta = invJ * err;
        temp += delta;
    }
    if (temp(0) < -1.0 || temp(0) > 1.0 || temp(1) < -1.57 || temp(1) > 1.57) {
        return false;
    }
    roll_pitch = temp;
    return true;
}

void FootLib::backward(const Eigen::Vector2d& roll_pitch, Eigen::Vector2d& upper_lower_theta) {
    foot_forward(roll_pitch(0), roll_pitch(1), upper_lower_theta.data());
}

Eigen::Matrix2d FootLib::jacobian(Eigen::Vector2d roll_pitch) {
    Eigen::Matrix2d j;
    foot_jacobian(roll_pitch(0), roll_pitch(1), j.data());
    return j;
}

}
