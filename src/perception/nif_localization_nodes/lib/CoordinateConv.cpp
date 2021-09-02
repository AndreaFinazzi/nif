#include "ekf_localizer/CoordinateConv.h"

CoordinateConv::CoordinateConv() {}

CoordinateConv::~CoordinateConv() {}

void CoordinateConv::OnParamInit() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    dWGS84DD_Lat[i] = NULL;
    dWGS84DD_Lon[i] = NULL;
  }

  dWGS84_DDLAT = 0;
  dWGS84_DDLON = 0;
}

void CoordinateConv::GPSWGS84_DM2DD(double Latitude, double Longitude) {
  double dLat_deg, dLon_deg;
  double dLat_degdec, dLon_degdec;
  double dLat_min, dLon_min;

  // ddxx.xxxxxx , dddxx.xxxxx
  dLat_deg = floor(Latitude / 100.0);
  dLon_deg = floor(Longitude / 100.0);
  // xxmm.mmmmmm , xxxmm.mmmmm
  dLat_min = Latitude - (dLat_deg * 100.0);
  dLon_min = Longitude - (dLon_deg * 100.0);

  dLat_degdec = (dLat_min / 60.0);
  dLon_degdec = (dLon_min / 60.0);

  dWGS84_DDLAT = dLat_deg + dLat_degdec;
  dWGS84_DDLON = dLon_deg + dLon_degdec;
}

void CoordinateConv::WGS2UTM(double Latitude, double Longitude) {
  double dLat, dLon;

  // coordinates in radians
  dLat = Latitude * M_PI / 180;
  dLon = Longitude * M_PI / 180;

  // UTM parameters
  double lon0_f =
      floor(Longitude / 6) * 6 + 3;  // reference longitude in degrees
  double lon0 = lon0_f * M_PI / 180; // in radians
  double k0 = 0.9996;                // scale on central meridian

  int FE = 500000;                    // false easting
  int FN = (Latitude < 0) * 10000000; // false northing

  // Equations parameters
  // N: radius of curvature of the earth perpendicular to meridian plane
  // Also, distance from point to polar axis
  double WN = Wa / sqrt(1 - pow(We, 2) * pow(sin(dLat), 2));
  double WT = pow(tan(dLat), 2);
  double WC = (pow(We, 2) / (1 - pow(We, 2))) * pow(cos(dLat), 2);
  double WLA = (dLon - lon0) * cos(dLat);
  // M: true distance along the central meridian from the equator to lat
  double WM =
      Wa *
      ((1 - pow(We, 2) / 4 - 3 * pow(We, 4) / 64 - 5 * pow(We, 6) / 256) *
           dLat -
       (3 * pow(We, 2) / 8 + 3 * pow(We, 4) / 32 + 45 * pow(We, 6) / 1024) *
           sin(2 * dLat) +
       (15 * pow(We, 4) / 256 + 45 * pow(We, 6) / 1024) * sin(4 * dLat) -
       (35 * pow(We, 6) / 3072) * sin(6 * dLat));

  // easting
  dUTM_X = FE + k0 * WN *
                    (WLA + (1 - WT + WC) * pow(WLA, 3) / 6 +
                     (5 - 18 * WT + pow(WT, 2) + 72 * WC - 58 * Weps) *
                         pow(WLA, 5) / 120);

  // northing
  // M(lat0) = 0 so not used in following formula
  dUTM_Y = FN + k0 * WM +
           k0 * WN * tan(dLat) *
               (pow(WLA, 2) / 2 +
                (5 - WT + 9 * WC + 4 * pow(WC, 2)) * pow(WLA, 4) / 24 +
                (61 - 58 * WT + pow(WT, 2) + 600 * WC - 330 * Weps) *
                    pow(WLA, 6) / 720);

  // UTM zone
  iUTM_zone = (int)(floor(lon0_f / 6) + 31);
}

/*
void CoordinateConv::UTM2WGS(double utmx, double utmy, int utmzone)
{
    double sa = 6378137.000000;
    double sb = 6356752.314245;

    double e2 = sqrt((sa*sa) - (sb*sb))*0.0000001573130351105668;
    double e2cuadrada = e2*e2;
    double cc = (sa*sa)*0.0000001573130351105668;

    // Utmzone > 'M' => Y = y, Utmzone < 'M' => Y = y - 10000000
    utmx = utmx - 500000;
    utmy = utmy;

    int Szt = ((utmzone*6) - 183);
    double latr =  utmy*0.0000001571424896673576;
    double vv = (cc/sqrt((1 + (e2cuadrada*(cos(latr))*(cos(latr))))))*0.9996;
    double aa = utmx/vv;
    double a1 = sin(2*latr);
    double a2 = a1*(cos(latr))*(cos(latr));
    double j2 = latr + (a1*0.5);
    double j4 = ((3*j2) + a2)*0.25;
    double j6 = ((5*j4) + (a2*(cos(latr))*(cos(latr))))*0.333333333333333;
    double alfa = 0.75*e2cuadrada;
    double beta = 1.666666666666667*alfa*alfa;
    double gama = 1.296296296296296*alfa*alfa*alfa;
    double Bm = 0.9996*cc*(latr - alfa*j2 + beta*j4 - gama*j6);
    double bb = (utmy - Bm)/vv;
    double Epsi = ((e2cuadrada*aa*aa)*0.5)*(cos(latr))*(cos(latr));
    double Eps = aa*(1 - (Epsi*0.333333333333333));
    double nab = (bb*(1 - Epsi)) + latr;
    double senoheps = (exp(Eps) - exp(-Eps))*0.5;
    double Delt = atan(senoheps/(cos(nab)));
    double TaO = atan(cos(Delt) * tan(nab));

    dWGS84_Lon = (Delt*(180/M_PI)) + Szt;
    dWGS84_Lat = (latr + (1 + e2cuadrada*(cos(latr)*cos(latr))
- 1.5*e2cuadrada*sin(latr)*cos(latr)*(TaO - latr))*(TaO - latr))*(180/M_PI);
}
*/
