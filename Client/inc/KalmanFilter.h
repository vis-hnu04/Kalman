#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_ KALMANFILTER_H_

#include<eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;


class KalmanFilter{
public:
 KalmanFilter();

 ~KalmanFilter();


private:
     VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // state transistion matrix
    MatrixXd F_;

    // process covariance matrix
    MatrixXd Q_;

    // measurement matrix
    MatrixXd H_;  

    // measurement covariance matrix
    MatrixXd R_;


};

#endif