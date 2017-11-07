#include<iostream>
#include<vector>
#include<Eigen/Dense>

using Eigen::MatrixXd;

class KalmanFilter {
public:

    //Matrices for Kalman filter
    MatrixXd X; //inital state (position and velocity)
    MatrixXd P; //intial uncertainity
    MatrixXd U; //External Motion
    MatrixXd F; //Next state fn
    MatrixXd H; //Measurement function
    MatrixXd R; //Measurement Uncertainity
    MatrixXd I; //Identity

    struct KalmanOutputStruct {
        MatrixXd x;
        MatrixXd p;
    };

    void set_matrix_values(MatrixXd _X, MatrixXd _P, MatrixXd _U,
                           MatrixXd _F, MatrixXd _H, MatrixXd _R,
                           MatrixXd _I);

    KalmanOutputStruct filter(std::vector<float> measurements);
};
