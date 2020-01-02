#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  vector <VectorXd> sqr_residuals;
  VectorXd r(4);
  
  // check the validity of inputs
  // the estimation vector size should not be zero
  if (estimations.size() == 0){
    cout << "CalculateRMSE () - Error - No estimations availble to compute RMSE"<< endl;
    return rmse;
  }
  // the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()){
    cout << "CalculateRMSE () - Error - Size of estimations and ground truth do not match"<< endl;
    return rmse;
  }
  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // calculate residuals
    r = estimations[i]-ground_truth[i];
    r = r.array()*r.array();
    //cout << r << endl;
    sqr_residuals.push_back(r);
  }  
  // calculate the mean
  for (int i=0; i < sqr_residuals.size(); ++i) {
    rmse = rmse + sqr_residuals[i];
  }
  rmse = rmse.array()/sqr_residuals.size();
  // calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float pxysqr = powf(px, 2) + powf(py, 2);
  float pxysqrrt =  hypotf(px, py);

  // check division by zero
  if (fabs(pxysqr) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }
  // compute the Jacobian matrix  
  Hj << px/pxysqrrt, py/pxysqrrt, 0, 0,
        -py/pxysqr, px/pxysqr, 0, 0,
        py*(vx*py-vy*px)/powf(pxysqrrt,3), px*(vy*px-vx*py)/powf(pxysqrrt,3), px/pxysqrrt, py/pxysqrrt;

  return Hj; 
}
