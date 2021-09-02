#include "ekf_localizer/c_ekf.h"

c_ekf::c_ekf() { EKF_Initialization(); }

c_ekf::~c_ekf() {
  m_xhat.release();
  m_Phat.release();
  m_Q.release();
  m_R1.release();
  m_R2.release();
  m_R3.release();
}

void c_ekf::EKF_Initialization() {
  m_xhat = Mat::zeros(3, 1, CV_64FC1);
  m_Phat = Mat::zeros(3, 3, CV_64FC1);

  m_Q = Mat::zeros(3, 3, CV_64FC1);

  // x, y, velocity, heading, yaw_rate
  m_Q.ptr<double>(0)[0] = 0.5;
  m_Q.ptr<double>(1)[1] = 0.5;
  m_Q.ptr<double>(2)[2] = 0.0341;

  m_R1 = Mat::zeros(3, 3, CV_64FC1);
  m_R2 = Mat::zeros(3, 3, CV_64FC1);
  m_R3 = Mat::zeros(3, 3, CV_64FC1);

  // gps single
  m_R1.ptr<double>(0)[0] = 10;
  m_R1.ptr<double>(1)[1] = 10;
  // m_R1.ptr<double>(2)[2] = 0.5;
  m_R1.ptr<double>(2)[2] = 2;

  // gps rtk-float
  m_R2.ptr<double>(0)[0] = 2;
  m_R2.ptr<double>(1)[1] = 2;
  m_R2.ptr<double>(2)[2] = 0.5;

  // gps rtk-fixed
  m_R3.ptr<double>(0)[0] = 0.5;
  m_R3.ptr<double>(1)[1] = 0.5;
  m_R3.ptr<double>(2)[2] = 0.5;
}

void c_ekf::EKF_Predictionstep(Mat xhat, Mat Phat, Mat odom, double dt) {
  double pos_x, pos_y, vel, heading, yaw_rate;
  double pos_x_prev, pos_y_prev, heading_prev;

  vel = odom.ptr<double>(0)[0];
  yaw_rate = odom.ptr<double>(1)[0];
  // vel_y = odom.ptr<double>(2)[0];
  pos_x_prev = xhat.ptr<double>(0)[0];
  pos_y_prev = xhat.ptr<double>(1)[0];
  heading_prev = xhat.ptr<double>(2)[0];

  heading = heading_prev + yaw_rate * dt;
  heading = AngDiff(heading);

  pos_x = pos_x_prev + vel * cos(heading) * dt;
  pos_y = pos_y_prev + vel * sin(heading) * dt;

  xhat.ptr<double>(0)[0] = pos_x;
  xhat.ptr<double>(1)[0] = pos_y;
  xhat.ptr<double>(2)[0] = heading;

  Mat A;
  A = Mat::zeros(3, 3, CV_64FC1);

  A.ptr<double>(0)[0] = 1;
  A.ptr<double>(0)[1] = 0;
  A.ptr<double>(0)[2] = -vel * sin(heading) * dt;
  A.ptr<double>(1)[0] = 0;
  A.ptr<double>(1)[1] = 1;
  A.ptr<double>(1)[2] = vel * cos(heading) * dt;
  A.ptr<double>(2)[0] = 0;
  A.ptr<double>(2)[1] = 0;
  A.ptr<double>(2)[2] = 1;

  // A.t() : Transpose
  Phat = A * Phat * A.t() + m_Q;

  m_Phat = Phat.clone();
  m_xhat = xhat.clone();
}

void c_ekf::EKF_Correctionstep(Mat xhat, Mat Phat, int gps_qual_indicator,
                               bool pos_flag, bool heading_flag,
                               Mat zMeasureSensor) {
  Mat zhat, I, H, StInverse;

  zhat = Mat::zeros(3, 1, CV_64FC1);
  zhat.ptr<double>(0)[0] = xhat.ptr<double>(0)[0];
  zhat.ptr<double>(1)[0] = xhat.ptr<double>(1)[0];
  zhat.ptr<double>(2)[0] = xhat.ptr<double>(2)[0];

  I = Mat::eye(3, 3, CV_64FC1);

  H = Mat::zeros(3, 3, CV_64FC1);

  H.ptr<double>(0)[0] = 1.0;
  H.ptr<double>(1)[1] = 1.0;
  H.ptr<double>(2)[2] = 1.0;

  if (gps_qual_indicator == 4) {
    StInverse = H * Phat * H.t() + m_R3;
  } else if (gps_qual_indicator == 5) {
    StInverse = H * Phat * H.t() + m_R2;
  } else {
    StInverse = H * Phat * H.t() + m_R1;
  }
  K = Phat * H.t() * StInverse.inv();
  if (pos_flag == false) {
    K.ptr<double>(0)[0] = 0.0;
    K.ptr<double>(1)[1] = 0.0;
  }
  if (heading_flag == true) {
    // cout << "kalman_gain" << K.ptr<double>(2)[2] << '\n';
  }
  if (heading_flag == false) {
    K.ptr<double>(2)[2] = 0.0;
    // cout << "kalman_gain" << K.ptr<double>(2)[2] << '\n';
  }
  if (fabs(zhat.ptr<double>(2)[0] - zMeasureSensor.ptr<double>(2)[0]) > M_PI) {
    if (zhat.ptr<double>(2)[0] - zMeasureSensor.ptr<double>(2)[0] > M_PI) {
      zMeasureSensor.ptr<double>(2)[0] =
          zMeasureSensor.ptr<double>(2)[0] + 2.0 * M_PI;
    } else {
      zMeasureSensor.ptr<double>(2)[0] =
          zMeasureSensor.ptr<double>(2)[0] - 2.0 * M_PI;
    }
  }
  xhat = xhat + K * (zMeasureSensor - zhat);
  Phat = (I - K * H) * Phat;

  m_xhat = xhat.clone();
  m_Phat = Phat.clone();
}

double c_ekf::AngDiff(double rad) {
  double retVal;
  double n;
  double cfmod;
  n = floor((rad + M_PI) / (2.0 * M_PI));
  cfmod = (rad + M_PI) - n * (2.0 * M_PI);

  retVal = cfmod - M_PI;

  return retVal;
}
