#include "kalman_filter.h"
#include <iostream>
#include <math.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
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
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	// extract position and velocity
	// x(0) -> px; x(1) -> py; x(2) -> vx; x(3) -> vy;

	// range: the distance to the pedestrian
	float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	// bearing: referenced counter-clockwise from the x-axis
	float phi = 0;
	if (fabs(x_[0]) > 0.001) {
	    phi = atan2(x_(1), x_(0));
	}

	// range rate: the projection of the velocity,
	float rho_dot = 0;
	// avoid divide by zero exception
	if (fabs(rho) > 0.0001) {
		rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
	}

	VectorXd z_pred(3);
	z_pred << rho, phi, rho_dot;
	VectorXd y = z - z_pred;
	if (fabs(y(1)) > 2 * M_PI) {
		std::cout << "phi in y = z - h(x): " << y(1) << std::endl;
		std::cout << "Normalizing Angles: atan2() returns values between -pi and pi.\n"
				"make sure phi in y = z - h(x) for radar measurements is in (-pi, pi)" << std::endl;
		y(1) = y(1) > 0 ? y(1) - 2 * M_PI : y(1) + 2 * M_PI;
	}

//	std::cout << x_(0) << " " << x_(1) << " " << x_(2) << " " << x_(3) << std::endl;
//	std::cout << "rho: " << rho << "phi: " << phi << "rho_dot: " << rho_dot << std::endl;
//	std::cout << z(0) << " " << z(1) << " " << z(2) << " \n" << std::endl;


	// have initialized H_ with Hj jacobian before
	MatrixXd Ht = H_.transpose();
//	std::cout << Ht << std::endl << std::endl;
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
