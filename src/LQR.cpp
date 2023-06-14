#include "LQR.h"
#include <iostream>

LQR::LQR(Eigen::Matrix<double, 4, 4> Q, Eigen::Matrix<double, 1, 1> R) {
    // Eigen::Matrix<double, 4, 4> S = ContinuousAlgebraicRicattiEquation(A, B, Q, R);
    K << -1/50, 51.06/50, -2.72/50, 12.83/50;
    std::cout << "LQR Gains: \n" << K << std::endl;
}

double LQR::getControl(Eigen::Vector4d state) {
    return -K.dot(state);
}