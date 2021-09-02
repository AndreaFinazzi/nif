/**
 * @brief utils to convert between geodetic frames and back
 *
 * @note Logic copied from https://github.com/ethz-asl/geodetic_utils/blob/master/geodetic_utils/include/geodetic_utils/geodetic_conv.hpp
 *  this gets slightly different results than pymap3d (https://github.com/geospace-code/pymap3d/tree/bd91a5ff4e9066eb33fead3006ba9de191e2c5e5/src/pymap3d)
 *  primarily in the z calculation - it might be worth comparing implementations (or swapping to python if that's desirable for whatever reason)
 *
 * @note this doesn't belong here.. should be a bvs_localization package for this to live in
 **/
#ifndef BVS_LOCALIZATION_UTILS_GEODETIC_CONV_H_
#define BVS_LOCALIZATION_UTILS_GEODETIC_CONV_H_

#include <eigen3/Eigen/Core>

namespace bvs_localization {
namespace utils {

// Geodetic system parameters (uses wgs-84)
const double kSemimajorAxis = 6378137;
const double kSemiminorAxis = 6356752.31424518;
const double kFirstEccentricitySquared = 6.69437999014 * 0.001;
const double kSecondEccentricitySquared = 6.73949674228 * 0.001;
const double kFlattening = 1 / 298.257223563;

/**
 * @brief converts between geodetic coords and different frames
 **/
class GeodeticConverter {
public:
    //! Geodetic Reference (GPS point)
    struct GeoRef {
        double latitude;
        double longitude;
        double altitude;
    };
    //! Cartesian Point in some frame
    struct CartesianPoint {
        double x;
        double y;
        double z;
    };

    /**
     * @brief construct a geodetic converter (no default reference point)
     **/
    GeodeticConverter();

    /**
     * @brief check if converter is initialized (has reference point)
     **/
    bool isInitialised() const;

    /**
     * @brief gets the reference point (only valid if isInitialized())
     * @param[out] ref the reference point
     **/
    void getReference(GeoRef& ref);

    /**
     * @brief set the reference point
     * @param ref the new referent point to use
     **/
    void initializeReference(const GeoRef ref);

    /**
     * @brief converts from geodetic coords to a ECEF frame
     * @param[in] ref the point to convert from
     * @param[out] ecef_pt the resulting point in a ECEF frame
     **/
    void geodetic2Ecef(const GeoRef ref, CartesianPoint& ecef_pt);
    
    /**
     * @brief converts from geodetic coords to a ECEF frame
     * @param[in] ecef_pt the point in ECEF to convert from
     * @param[out] ref the resulting geodetic frame
     **/
    void ecef2Geodetic(const CartesianPoint ecef_pt, GeoRef& ref);

    /**
     * @brief converts from geodetic coords to a Ned frame
     * @param[in] ref the point to convert from
     * @param[out] ned_pt the resulting point in a NED frame
     **/
    void geodetic2Ned(const GeoRef ref, CartesianPoint& ned_pt);
    
    /**
     * @brief converts from geodetic coords to a Ned frame
     * @param[in] ned_pt the point in NED to convert from
     * @param[out] ref the resulting geodetic frame
     **/
    void ned2Geodetic(const CartesianPoint ned_pt, GeoRef& ref);

    /**
     * @brief convert from radians to degrees
     * @param radians angle in radians
     * @return angle in degrees
     **/
    double rad2Deg(const double radians);

    /**
     * @brief convert from degrees to radians
     * @param degrees angle in degrees
     * @return angle in radians
     **/
    double deg2Rad(const double degrees);

 private:
    //! I don't know what this does hahaha
    Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians);

    //! Whether or not we have a reference point
    bool have_reference_;

    //! Geodetic Reference Point
    GeoRef initial_geo_ref_;

    //! ECEF Reference Point
    CartesianPoint initial_ecef_;

    //! Convert between ECEF to NED
    Eigen::Matrix3d ecef_to_ned_matrix_;

    //! Convert from NED to ECEF
    Eigen::Matrix3d ned_to_ecef_matrix_;

}; // class GeodeticConverter

} /* namespace utils */
} /* namespace bvs_localization */

#endif // BVS_LOCALIZATION_UTILS_GEODETIC_CONV_H_