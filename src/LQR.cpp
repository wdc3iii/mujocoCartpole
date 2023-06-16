#include "LQR.h"
#include <iostream>

LQR::LQR(Eigen::Matrix<double, 4, 4> Q, Eigen::Matrix<double, 1, 1> R) {
    // Eigen::Matrix<double, 4, 4> S = ContinuousAlgebraicRicattiEquation(A, B, Q, R);
    K << -1/10, 51.06/10, -2.72/10, 12.83/10;
    std::cout << "LQR Gains: \n" << K << std::endl;
}

double LQR::getControl(Eigen::Vector4d state) {
    return K.dot(state);
}