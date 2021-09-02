#include "utils/geodetic_conv.h"

#include <cmath>
#include <iostream>

using namespace nif::localization::utils;

GeodeticConverter::GeodeticConverter() : have_reference_(false) {}

bool GeodeticConverter::isInitialised() { return have_reference_; }

void GeodeticConverter::getReference(GeodeticConverter::GeoRef &ref) {
  ref = initial_geo_ref_;
}

void GeodeticConverter::initializeReference(
    const GeodeticConverter::GeoRef ref) {
  initial_geo_ref_.latitude = deg2Rad(ref.latitude);
  initial_geo_ref_.longitude = deg2Rad(ref.longitude);
  initial_geo_ref_.altitude = ref.altitude;

  geodetic2Ecef(ref, initial_ecef_);
  double phiP =
      std::atan2(initial_ecef_.z, std::sqrt(std::pow(initial_ecef_.x, 2) +
                                            std::pow(initial_ecef_.y, 2)));

  ecef_to_ned_matrix_ = nRe(phiP, initial_geo_ref_.longitude);
  ned_to_ecef_matrix_ =
      nRe(initial_geo_ref_.latitude, initial_geo_ref_.longitude).transpose();

  have_reference_ = true;
}

void GeodeticConverter::geodetic2Ecef(const GeoRef ref,
                                      CartesianPoint &ecef_pt) {
  // Convert geodetic coordinates to ECEF.
  // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
  double lat_rad = deg2Rad(ref.latitude);
  double lon_rad = deg2Rad(ref.longitude);
  double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
  ecef_pt.x =
      (kSemimajorAxis / xi + ref.altitude) * cos(lat_rad) * cos(lon_rad);
  ecef_pt.y =
      (kSemimajorAxis / xi + ref.altitude) * cos(lat_rad) * sin(lon_rad);
  ecef_pt.z =
      (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + ref.altitude) *
      sin(lat_rad);
}

void GeodeticConverter::ecef2Geodetic(const CartesianPoint ecef_pt,
                                      GeoRef &ref) {
  // Convert ECEF coordinates to geodetic coordinates.
  // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
  // to geodetic coordinates," IEEE Transactions on Aerospace and
  // Electronic Systems, vol. 30, pp. 957-961, 1994.

  double r = sqrt(ecef_pt.x * ecef_pt.x + ecef_pt.y * ecef_pt.y);
  double Esq =
      kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
  double F = 54 * kSemiminorAxis * kSemiminorAxis * ecef_pt.z * ecef_pt.z;
  double G = r * r + (1 - kFirstEccentricitySquared) * ecef_pt.z * ecef_pt.z -
             kFirstEccentricitySquared * Esq;
  double C =
      (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) /
      pow(G, 3);
  double S = cbrt(1 + C + sqrt(C * C + 2 * C));
  double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
  double Q =
      sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
  double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q) +
               sqrt(0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q) -
                    P * (1 - kFirstEccentricitySquared) * ecef_pt.z *
                        ecef_pt.z / (Q * (1 + Q)) -
                    0.5 * P * r * r);
  double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) +
                  ecef_pt.z * ecef_pt.z);
  double V = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) +
                  (1 - kFirstEccentricitySquared) * ecef_pt.z * ecef_pt.z);
  double Z_0 =
      kSemiminorAxis * kSemiminorAxis * ecef_pt.z / (kSemimajorAxis * V);
  ref.altitude =
      U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
  ref.latitude =
      rad2Deg(atan((ecef_pt.z + kSecondEccentricitySquared * Z_0) / r));
  ref.longitude = rad2Deg(atan2(ecef_pt.y, ecef_pt.x));
}

void GeodeticConverter::geodetic2Ned(const GeoRef ref, CartesianPoint &ned_pt) {
  CartesianPoint ecef;
  geodetic2Ecef(ref, ecef);

  Eigen::Vector3d vect, ret;
  vect(0) = ecef.x - initial_ecef_.x;
  vect(1) = ecef.y - initial_ecef_.y;
  vect(2) = ecef.z - initial_ecef_.z;
  ret = ecef_to_ned_matrix_ * vect;
  ned_pt.x = ret(0);
  ned_pt.y = ret(1);
  ned_pt.z = -ret(2);
}

void GeodeticConverter::ned2Geodetic(const CartesianPoint ned_pt, GeoRef &ref) {
  // NED (north/east/down) to ECEF coordinates
  Eigen::Vector3d ned, ret;
  ned(0) = ned_pt.x;
  ned(1) = ned_pt.y;
  ned(2) = -ned_pt.z;
  ret = ned_to_ecef_matrix_ * ned;
  CartesianPoint ecef;
  ecef.x = ret(0) + initial_ecef_.x;
  ecef.y = ret(1) + initial_ecef_.y;
  ecef.z = ret(2) + initial_ecef_.z;

  ecef2Geodetic(ecef, ref);
}

Eigen::Matrix3d GeodeticConverter::nRe(const double lat_radians,
                                       const double lon_radians) {
  const double sLat = sin(lat_radians);
  const double sLon = sin(lon_radians);
  const double cLat = cos(lat_radians);
  const double cLon = cos(lon_radians);

  Eigen::Matrix3d ret;
  ret(0, 0) = -sLat * cLon;
  ret(0, 1) = -sLat * sLon;
  ret(0, 2) = cLat;
  ret(1, 0) = -sLon;
  ret(1, 1) = cLon;
  ret(1, 2) = 0.0;
  ret(2, 0) = cLat * cLon;
  ret(2, 1) = cLat * sLon;
  ret(2, 2) = sLat;

  return ret;
}

double GeodeticConverter::rad2Deg(const double radians) {
  return (radians / M_PI) * 180.0;
}

double GeodeticConverter::deg2Rad(const double degrees) {
  return (degrees / 180.0) * M_PI;
}