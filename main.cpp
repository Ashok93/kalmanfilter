#include <iostream>
#include<Eigen/Dense>
#include<vector>
#include "kalman_filter.h"

using Eigen::MatrixXd;

int main()
{
    std::vector<float> measurements = {1, 2, 3};

    //Kalman filter matrices
    MatrixXd X(2,1); //inital state (position and velocity)
    X(0,0) = 0.0;
    X(1,0) = 0.0;
    MatrixXd P(2,2); //intial uncertainity
    P(0,0) = 1000.0;
    P(0,1) = 0.0;
    P(1,0) = 0.0;
    P(1,1) = 1000.0;
    MatrixXd U(2,1); //External Motion
    U(0,0) = 0.0;
    U(1,0) = 0.0;
    MatrixXd F(2,2); //Next state fn
    F(0,0) = 1.0;
    F(0,1) = 1.0;
    F(1,0) = 0.0;
    F(1,1) = 1.0;
    MatrixXd H(1,2); //Measurement function
    H(0,0) = 1.0;
    H(0,1) = 0.0;
    MatrixXd R(1,1); //Measurement Uncertainity
    R(0,0) = 1.0;
    MatrixXd I(2,2); //Identity
    I(0,0) = 1.0;
    I(0,1) = 0.0;
    I(1,0) = 0.0;
    I(1,1) = 1.0;

    KalmanFilter kf;
    kf.set_matrix_values(X, P, U, F, H, R, I);

    KalmanFilter::KalmanOutputStruct predictions = kf.filter(measurements);
    std::cout<< "X" << std::endl << predictions.x << std::endl;
    std::cout<< "P" << std::endl << predictions.p << std::endl;

}
