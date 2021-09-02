/*
 *  linux-Coordinate Conversion class Lat, Long->UTM
 *  date : 2019.01.04
 *  made by Taejin Kim - KAIST_USRG
 *
 *  For God so loved the world that he gave his one and only Son,
 *  that whoever believes in him shall not perish but have eternal life.
 *  John 3:16
 */
#ifndef COORDINATECONV_H
#define COORDINATECONV_H

#ifndef NULL
#define NULL 0
#endif

#include <math.h>
// WGS84 parameters
#define UTM_ZON     52
#define Wa          6378137
#define Wb          6356752.314245
#define We          0.081819190842965
#define Weps        0.006739496742333

#define M_PI          3.1415926535897932384626433832795
#define BUFFER_SIZE 255	// GPS data size

class CoordinateConv
{
public:
    CoordinateConv();
    virtual ~CoordinateConv();

// Attributes
public:
    void OnParamInit();

    void GPSWGS84_DM2DD(double Latitude, double Longitude);
    void WGS2UTM(double Latitude, double Longitude);    

    double dWGS84DD_Lat[BUFFER_SIZE] ;
    double dWGS84DD_Lon[BUFFER_SIZE] ;
    double dWGS2UTM_X[BUFFER_SIZE] ;
    double dWGS2UTM_Y[BUFFER_SIZE] ;
    double dUTM_X, dUTM_Y, dWGS84_Lat, dWGS84_Lon;
    double dWGS84_DDLAT ;
    double dWGS84_DDLON ;
    int iUTM_zone;
};

#endif // COORDINATECONV_H
