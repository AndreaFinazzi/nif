#ifndef COMMON_DEF_H
#define COMMON_DEF_H

#define GRAVITY 9.80665

struct IMU_data
{
    double angular_vel_x;
    double angular_vel_y;
    double angular_vel_z;

    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;

    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;

    double euler_angle_x;
    double euler_angle_y;
    double euler_angle_z;
};

struct GPS_data
{
    int time;
    double latitude;
    char lat_direction;
    double longitude;
    char long_direction;
    int quality_indicator;
    int num_use_satellite;
    double HDOP; //deviation
    double height;
    int rmc_time;
    double gprmc_speed;
    double gprmc_gps_heading; //degree
    double pos_x;
    double pos_y;
};

struct ODOM_data
{
    double velocity;
};

struct Cartesian
{
    double x;
    double y;
    double z;
};

struct Posture
{
    double x;
    double y;
    double heading;
};

struct Euler
{
    double roll;
    double pitch;
    double yaw;
};

struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

class COMMON_DEF
{
public:
    COMMON_DEF() {}

    IMU_data m_imu_data;
    GPS_data m_gps_data;

    Euler e;

    Quaternion temp_variable;
};

#endif // COMMON_DEF_H
