#ifndef FOOT_KINEMATICS_H
#define FOOT_KINEMATICS_H

namespace humanoid {
    struct FootKinematics {
        static bool forward(double upper_theta, double lower_theta, double& roll, double& pitch, double tolerance=0.001, int max_iter=10);
        static void backward(double roll, double pitch, double& upper_theta, double& lower_theta);
    };
}


#endif // FOOT_KINEMATICS_H
