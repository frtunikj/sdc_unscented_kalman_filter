#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <iomanip> // for std::setprecision

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void NormalizeAngle(double& phi) {
    phi = atan2(sin(phi), cos(phi));
}

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // If true, the UKF has been initialized
    is_initialized_ = false;

    time_us_ = 0.0;

    // State dimension (px, py, v, yaw, yawd)
    n_x_ = 5;
    // Augmented state dimension (px, py, v, yaw, yawd, nu_a, nu_yawdd)
    n_aug_ = 7;
    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;

    NIS_radar_ = 0.0;
    NIS_lidar_ = 0.0;

    // Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);

    weights_(0) = lambda_ / (lambda_ + n_aug_);

    for (int i = 1; i < 2 * n_aug_ + 1; i++) { // 2n+1 weights
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }

    // initial state vector
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    // initial augmented sigma points
    Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // initial predicted augmented sigma point matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    n_z_lidar_ = 2;
    z_pred_lidar_ = VectorXd(n_z_lidar_);
    S_pred_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
    Zsig_pred_lidar_ = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);

    n_z_radar_ = 3;
    z_pred_radar_ = VectorXd(n_z_radar_);
    S_pred_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
    Zsig_pred_radar_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

}

UKF::~UKF() {
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

    if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) ||
            (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)) {

        //-------------------------------------------------------
        // Initialize UKF with first measurement
        //-------------------------------------------------------

        if (!is_initialized_) {
            // init timestamp
            time_us_ = meas_package.timestamp_;

            // init covariance matrix
            P_ << 0.15, 0, 0, 0, 0,
                    0, 0.15, 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1;

            if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
                // init measurement with LIDAR data [x, y] ==> [px, py, v, yaw, yawd]
                x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 1, 1, 0.1;
                std::cout << fixed << "Init state from Radar @timestamp " << std::setprecision(3) << time_us_ / (double) 1e6 << std::endl << std::endl;
            } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
                // init measurement with RADAR data [rho, phi, rho_dot] ==> [px, py, v, yaw, yawd]
                float rho = meas_package.raw_measurements_(0);
                float phi = meas_package.raw_measurements_(1);
                x_ << rho * cos(phi), rho * sin(phi), 1, 1, 0.1;
                std::cout << fixed << "Init state from Radar @timestamp " << std::setprecision(3) << time_us_ / (double) 1e6 << std::endl << std::endl;
            }

            // initialization performed
            is_initialized_ = true;
            return;
        }

        //-------------------------------------------------------
        // UKF prediction step
        //-------------------------------------------------------

        cout << fixed << "Previous Timestamp = " << time_us_ / (double) 1e6 << endl;

        //dt - expressed in seconds
        float dt = (meas_package.timestamp_ - time_us_) / (double) 1e6;
        time_us_ = meas_package.timestamp_;

        std::string step_name = "STATE predicrion";
        Prediction(dt);

        //-------------------------------------------------------
        // UKF update step
        //-------------------------------------------------------

        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            step_name = "LIDAR prediction update";
            PredictLidarMeasurement(meas_package);
            UpdateLidar(meas_package);
        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            step_name = "RADAR prediction update";
            PredictRadarMeasurement(meas_package);
            UpdateRadar(meas_package);
        }

        cout << "------------------------------   " << step_name << endl;
        cout << "Timestamp = " << std::setprecision(3) << time_us_ / (double) 1e6 << endl;
        cout << "x_ = " << endl << x_ << endl;
        cout << "P_ = " << endl << P_ << endl << endl;
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
     */
    // Generate augmented sigma points Xsig_aug_
    GenerateAugmentedSigmaPoints();

    // Predict augmented sigma points Xsig_pred_
    PredictSigmaPoints(delta_t);

    // Predict state vector x_ and state covariance matrix P_
    PredictMeanAndCovariance();
}

/**
 * Generates the augmented sigma points matrix.
 */

void UKF::GenerateAugmentedSigmaPoints() {

    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0.0;
    x_aug(6) = 0.0;

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = std_a_*std_a_;
    P_aug(6, 6) = std_yawdd_*std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug_.fill(0.0);
    Xsig_aug_.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i) {
        Xsig_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }
}

/**
 * Predict the augmented sigma points matrix.
 *
 * @param {double} delta_t the change in time (in seconds) between the last
 *        measurement and this one.
 */
void UKF::PredictSigmaPoints(double delta_t) {

    //predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // extract values for better readability
        double p_x = Xsig_aug_(0, i);
        double p_y = Xsig_aug_(1, i);
        double v = Xsig_aug_(2, i);
        double yaw = Xsig_aug_(3, i);
        double yawd = Xsig_aug_(4, i);
        double nu_a = Xsig_aug_(5, i);
        double nu_yawdd = Xsig_aug_(6, i);

        // predicted state values
        double px_p, py_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        } else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // add noise
        px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p += nu_a * delta_t;

        yaw_p += 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p += nu_yawdd * delta_t;

        // write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
}

/**
 * Predict state vector and state covariance matrix.
 */
void UKF::PredictMeanAndCovariance() {

    // predicted state mean
    x_.fill(0.0);

    // iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);

    // iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {

        // state difference
        VectorXd x_diff = (Xsig_pred_.col(i) - x_);

        //angle normalization
        NormalizeAngle(x_diff(3));

        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::PredictLidarMeasurement(MeasurementPackage meas_package) {

    // Transform sigma points from state space to measurement space

    for (int i = 0; i < Zsig_pred_lidar_.cols(); i++) {

        // extract values
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);

        // measurement model
        Zsig_pred_lidar_(0, i) = p_x; // x
        Zsig_pred_lidar_(1, i) = p_y; // y
    }

    // mean predicted measurement
    z_pred_lidar_.fill(0.0);

    for (int i = 0; i < Zsig_pred_lidar_.cols(); i++) {
        z_pred_lidar_ += weights_(i) * Zsig_pred_lidar_.col(i);
    }

    // measurement covariance matrix S
    S_pred_lidar_.fill(0.0);

    for (int i = 0; i < Zsig_pred_lidar_.cols(); i++) {

        // residual
        VectorXd z_diff = Zsig_pred_lidar_.col(i) - z_pred_lidar_;

        // angle normalization
        NormalizeAngle(z_diff(1));

        S_pred_lidar_ += weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z_lidar_, n_z_lidar_);

    R << std_laspx_ * std_laspx_, 0,
            0, std_laspy_ * std_laspy_;

    S_pred_lidar_ += R;
}

/**
/ * Updates the state and the state covariance matrix using a laser measurement.
/ * @param {MeasurementPackage} meas_package
/ */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.
    Calculate the lidar NIS.
     */
    VectorXd z_lidar(n_z_lidar_);
    z_lidar = meas_package.raw_measurements_;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < Zsig_pred_lidar_.cols(); i++) {

        VectorXd z_diff = Zsig_pred_lidar_.col(i) - z_pred_lidar_;

        // angle normalization
        NormalizeAngle(z_diff(1));

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // angle normalization
        NormalizeAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S_pred_lidar_.inverse();

    // residual
    VectorXd z_diff = z_lidar - z_pred_lidar_;

    //angle normalization
    NormalizeAngle(z_diff(1));

    // calculate NIS
    NIS_lidar_ = tools_.CalculateNIS(z_lidar, z_pred_lidar_, S_pred_lidar_);

    //update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S_pred_lidar_ * K.transpose();

}

void UKF::PredictRadarMeasurement(MeasurementPackage meas_package) {

    // extract radar measurements (r, phi, r_dot)
    VectorXd z_radar(n_z_radar_);
    z_radar = meas_package.raw_measurements_;

    // transform sigma points into measurement space
    for (int i = 0; i < Zsig_pred_radar_.cols(); i++) {

        // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig_pred_radar_(0, i) = sqrt(p_x * p_x + p_y * p_y); // r
        Zsig_pred_radar_(1, i) = atan2(p_y, p_x); // phi
        Zsig_pred_radar_(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); // r_dot
    }

    // mean predicted measurement
    z_pred_radar_.fill(0.0);

    for (int i = 0; i < Zsig_pred_radar_.cols(); i++) {
        z_pred_radar_ = z_pred_radar_ + weights_(i) * Zsig_pred_radar_.col(i);
    }

    // measurement covariance matrix S
    S_pred_radar_.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) { 

        VectorXd z_diff = Zsig_pred_radar_.col(i) - z_pred_radar_; 

        // angle normalization
        NormalizeAngle(z_diff(1));

        S_pred_radar_ += weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z_radar_, n_z_radar_);

    R << std_radr_ * std_radr_, 0, 0,
            0, std_radphi_ * std_radphi_, 0,
            0, 0, std_radrd_ * std_radrd_;
    S_pred_radar_ += R;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.
    Calculate the radar NIS.
     */
    
    VectorXd z_radar(n_z_radar_);
    z_radar = meas_package.raw_measurements_;
    
    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < Zsig_pred_radar_.cols(); i++) { 

        VectorXd z_diff = Zsig_pred_radar_.col(i) - z_pred_radar_; 

        // angle normalization
        NormalizeAngle(z_diff(1));

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // angle normalization
        NormalizeAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S_pred_radar_.inverse();

    // residual
    VectorXd z_diff = z_radar - z_pred_radar_;

    //angle normalization
    NormalizeAngle(z_diff(1));

    // calculate NIS
    NIS_radar_ = tools_.CalculateNIS(z_radar, z_pred_radar_, S_pred_radar_);

    //update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S_pred_radar_ * K.transpose();
}
