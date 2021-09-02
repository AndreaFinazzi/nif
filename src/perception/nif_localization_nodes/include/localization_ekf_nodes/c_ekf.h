#ifndef C_EKF_H
#define C_EKF_H

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

using namespace cv;
using namespace std;

class c_ekf {
public:
  c_ekf();
  ~c_ekf();

  Mat m_xhat;
  Mat m_Phat;
  Mat m_Q;
  Mat m_R1;
  Mat m_R2;
  Mat m_R3;
  Mat K;

  void EKF_Initialization();
  void EKF_Predictionstep(Mat xhat, Mat Phat, Mat odom, double dt);
  void EKF_Correctionstep(Mat xhat, Mat Phat, int gps_qual_indicator,
                          bool pos_flag, bool heading_flag, Mat zMeasureSensor);

  double AngDiff(double rad);
};

#endif // C_EKF_H
