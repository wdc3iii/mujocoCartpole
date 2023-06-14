#ifndef lqr_h
#define lqr_h
#include "controller.h"

class LQR: public Controller {
    private:
    Eigen::Vector4d K;

    public:
    LQR(Eigen::Matrix<double, 4, 4> Q, Eigen::Matrix<double, 1, 1> R);
    double getControl(Eigen::Vector4d state);
};

#endif