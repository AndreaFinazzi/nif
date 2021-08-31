#ifndef BVS_CONTROL_LQR_TIERED_LATERAL_LQR_H_
#define BVS_CONTROL_LQR_TIERED_LATERAL_LQR_H_

#include "eigen3/Eigen/Core"
#include <memory>
#include <vector>

namespace bvs_control {
namespace lqr {

/**
 * Lateral LQR feedback control with multiple feedback matrices
 * varying by speed
 **/
class LateralLQR {
public:
    //! State Matrix (Assumes FLU coordinate frame)
    //! (0,0) = x           (world frame)
    //! (1,0) = y           (world frame)
    //! (2,0) = x dot       (body frame velocity)
    //! (3,0) = y dot       (body frame velocity)
    //! (4,0) = theta       (world frame)
    //! (5,0) = theta dot   (world frame)
    typedef Eigen::Matrix<double, 6,1> StateMatrix;

    //! Goal Matrix (Same coordinate frame as state)
    //! (0,0) = x
    //! (0,0) = y
    //! (0,0) = yaw
    typedef Eigen::Matrix<double, 3,1> GoalMatrix;

    //! The error state matrix
    //! (0,0) = lateral error
    //! (1,0) = lateral velocity error
    //! (2,0) = yaw error
    //! (3,0) = yaw dot error
    typedef Eigen::Matrix<double, 4, 1> ErrorMatrix;

    //! The actual feedback matrix
    typedef Eigen::Matrix<double, 1, 4> KMatrix;
    //! An array of kmatrices with velocity tiers
    typedef std::vector<std::pair<double, KMatrix>> KMatrices;

    //! Shared ptr..
    typedef std::shared_ptr<LateralLQR> Ptr;
    //! Create a new pointer (takes same args as any constructor)
    template<typename... Arguments>
    static Ptr newPtr(Arguments... args) {
        return std::make_shared<LateralLQR>(args...);
    }

    /**
     * @brief constructs a lateral lqr controller
     * @param config_file must have a valid default config
     *  file to load from (even if you can use loadConfig 
     *  or getKMatrices() to overwrite them)
     **/
    LateralLQR(std::string config_file);

    /**
     * @brief generates the feedback matrices from the config file
     * @param config_file the file to load
     **/
    void loadConfig(std::string config_file);

    /**
     * @brief computes an error matrix from the current state and a goal
     * @param state the current vehicle state
     * @param goal the goal to track to
     * @return ErrorMatrix
     *
     * @note this assumes that the target yaw rate is 0, this could
     *  be improved by calculating the target yaw rate from path
     *  curvature, I just didn't get the chace to implement / test
     **/
    ErrorMatrix computeError(StateMatrix state, GoalMatrix goal);

    /**
     * @brief processes the error at a given velocity to compute the
     *  optimal steering response
     * @param error state error matrix
     * @param x_vel_body the velocity of the vehicle in the x direction (body frame)
     * @return double steering response
     **/
    double process(ErrorMatrix error, double x_vel_body);
    
    /**
     * @brief processes the current vehicle state and a goal to compute
     *  an optimal steering response
     * @param state the current vehicle state
     * @param goal a goal for the vehicle to track to
     * @return double steering response
     **/
    double process(StateMatrix state, GoalMatrix goal);

    /**
     * @brief direct accessor for the controllers feedback matrix
     *  this can be used either to set matrices or get the values
     *  (mostly provided for testing)
     * @return KMatrices the feedback matrices (and their velocity tiers)
     **/
    KMatrices& getKMatrices();
private:
    /**
     * @brief wraps an angle to be between -PI and PI
     * @param theta any angle
     * @return the same angle bounded to -PI and PI
     **/
    double wrap_angle(double theta);

    //! feedback matrices
    std::vector<std::pair<double, KMatrix>> matrices_;

}; /* class LateralLQR */

} /* namespace lqr */
} /* namespace bvs_control */

#endif /* BVS_CONTROL_LQR_TIERED_LATERAL_LQR_H_ */