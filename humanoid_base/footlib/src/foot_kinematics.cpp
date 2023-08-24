#include "foot_kinematics.h"
#include "foot_forward.h"
#include "foot_jacobian_inverse.h"
#include <cmath>

void humanoid::FootKinematics::forward(double upper_theta, double lower_theta, double &roll, double &pitch, double tolerance, int max_iter) {
    double roll_temp = 0;
    double pitch_temp = 0;
    for (int i = 0; i < max_iter; ++i) {
        double yhat[2];
        double invJ[4];
        double err[2];
        double delta[2];
        double total_err;

        foot_forward(roll_temp, pitch_temp, yhat);
        foot_jacobian_inverse(roll_temp, pitch_temp, invJ);

        err[0] = upper_theta - yhat[0];
        err[1] = lower_theta - yhat[1];
        total_err = fabs(err[0]) + fabs(err[1]);

        if (total_err < tolerance)
            break;

        for (int j = 0; j < 2; ++j) {
            delta[j] = invJ[j] * err[0] + invJ[j + 2] * err[1];
        }

        roll_temp += delta[0];
        pitch_temp += delta[1];
    }
    roll = -roll_temp;
    pitch = pitch_temp;
}

void humanoid::FootKinematics::backward(double roll, double pitch, double &upper_theta, double &lower_theta) {
    double y[2];
    foot_forward(roll, pitch, y);
    upper_theta = y[0];
    lower_theta = y[1];
}
