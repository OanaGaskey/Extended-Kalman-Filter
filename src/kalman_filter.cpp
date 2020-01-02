#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;	
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  // transform predicted state from cartesian to polar coordinates
  // h fuction is applied since the transformation is non linear
  float zp1 = hypotf(px, py);
  float zp2 = atan2(py,px);
  if (fabs(zp1) < 0.0001) {
    cout << "UpdateEKF () - Error - Division by Zero" << endl;
    return;
  }
  float zp3 = (px*vx + py*vy)/zp1;
  // assembly predicted z vector in polar coordinates
  VectorXd z_pred(3);
  z_pred << zp1, zp2, zp3;
  VectorXd y = z - z_pred;
  // normalizing phi
  // phi angle should be in -pi,+pi range after the substraction is done
  const float  PI_F = 3.14159265358979f;
  if (y(1) > PI_F){
  	y(1) = y(1) - (2.0f)*PI_F;
  }
  if (y(1) < (-1.0f)*PI_F){
  	y(1) = y(1) + (2.0f)*PI_F;
  }
  
  // calculate the Extended Kalman Filter matrices using the Jacobian to linearize the measurement function
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
