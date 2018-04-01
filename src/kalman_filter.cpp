#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
    // Initialize state covariance matrix P
    P_ = MatrixXd(4, 4);
    F_ = MatrixXd(4, 4);
    Q_ = MatrixXd(4, 4);
    
    P_ << 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1000.0, 0.0,
          0.0, 0.0, 0.0, 1000.0;

    // Initial transition matrix F_
    F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // Initialize process noise covariance matrix
    Q_ << 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0;
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
     * predict the state
     */
    cout << "KF predict: " << endl;

    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     * update the state by using Kalman Filter equations
     */
    VectorXd y;
    
    cout << "KF update: " << endl;
    y = z - H_ * x_;
    UpdateState(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     * update the state by using Extended Kalman Filter equations
     */
    VectorXd y(3), z_predicted(3);

    cout << "EKF update: " << endl;

    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    // rho
    z_predicted(0) = sqrt(px * px + py * py); 
    // phi
    z_predicted(1) = atan2(py, px); 
    // rho-dot, avoid division by zero
    z_predicted(2) = (px * vx + py * vy) / std::max(0.00001, z_predicted(0)); 

    y = z - z_predicted;

    // ensure phi is within range [-pi, pi]
    NormalizeAngle(y(1));

    UpdateState(y);
}

void KalmanFilter::UpdateState(const VectorXd &y) {
    
    MatrixXd S, K, H_t;
    MatrixXd I = MatrixXd::Identity(4, 4);

    cout << "KF update state: " << endl;

    H_t = H_.transpose();

    S = H_ * P_ * H_t + R_;

    // Kalman gain
    K = P_ * H_t * S.inverse();

    // Measurement update step
    x_ = x_ + K * y; 
    P_ -= K * H_ * P_;
}


