#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  VectorXd diff(4), rmse(4);
  diff << 0,0,0,0;
  rmse << 0, 0, 0, 0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
  
  // accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    
    diff = estimations[i] - ground_truth[i];
    diff = diff.array() * diff.array();

    rmse += diff;
  }
  
  //calculate the mean and squared root
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

/**
 * Calculates the Normalized Innovation Squared (NIS) value.
 *
 * @param measurements      Measurement z_k+1.
 * @param pred_measurements Predicted measurement z_k+1|k
 * @param S                 Predicted measurement covariance matrix S_k+1|k.
 *
 * @return NIS value.
 */
double Tools::CalculateNIS(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S) {
  
  VectorXd z_diff = z - z_pred;
  double e = z_diff.transpose() * S.inverse() * z_diff;
  
  return e;
}