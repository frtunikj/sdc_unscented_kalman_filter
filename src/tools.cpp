#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
}

Tools::~Tools() {
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
        const vector<VectorXd> &ground_truth) {
    /**
     * Calculate the RMSE here.
     */
    VectorXd diff(4), rmse(4);
    diff << 0, 0, 0, 0;
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size()) {
        throw std::runtime_error("Estimations and ground truth vectors must have the same size!");
    } else if (estimations.size() == 0) {
        throw std::runtime_error("Estimations vector has zero length!");
    } else if (ground_truth.size() == 0) {
        throw std::runtime_error("Ground truth vector has zero length!");}

    for (std::size_t i = 0; i < estimations.size(); ++i) {
        diff = estimations[i] - ground_truth[i];
        diff = diff.array() * diff.array();

        rmse += diff;
    }

    rmse = rmse / estimations.size();
    return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd & x_state) {
    /**
     * Calculate a Jacobian here.
     */
    MatrixXd H_j(3, 4);
    
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float ms = px * px + py*py;
    float rms = sqrt(ms);
    float rms_cubed = ms*rms;

    // Check for division by zero
    if (rms < std::numeric_limits<float>::epsilon()) {
        throw std::runtime_error("Division by zero!");
    }

    H_j << px / rms, py / rms, 0, 0,
            -py / ms, px / ms, 0, 0,
            py * (vx * py - vy * px) / rms_cubed, px * (vy * px - vx * py) / rms_cubed, px / rms, py / rms;

    return H_j;
}
