#include<iostream>
#include<vector>
#include<Eigen/Dense>
#include "kalman_filter.h"

using Eigen::MatrixXd;

void KalmanFilter::set_matrix_values(MatrixXd _X, MatrixXd _P, MatrixXd _U,
                                MatrixXd _F, MatrixXd _H, MatrixXd _R,
                                MatrixXd _I)
{
    X = _X;
    P = _P;
    U = _U;
    F = _F;
    H = _H;
    R = _R;
    I = _I;
}

KalmanFilter::KalmanOutputStruct KalmanFilter::filter(std::vector<float> measurements)
{
    KalmanFilter::KalmanOutputStruct kalman_output;

    for(auto &measurement : measurements)
    {
        MatrixXd measurement_mat(1,1);
        measurement_mat(0,0) = measurement;

        //measurement update
        MatrixXd y = measurement_mat - (H*X);
        MatrixXd s = H*P*H.transpose() + R;
        MatrixXd k = P*H.transpose() * s.inverse();
        X = X + k*y;
        P = (I - k*H)*P;

        //prediction
        X = F*X + U;
        P = F*P*F.transpose();
    }

    kalman_output.x = X;
    kalman_output.p = P;

    return kalman_output;

}
