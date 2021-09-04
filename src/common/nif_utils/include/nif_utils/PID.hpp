//
// Created by usrg on 9/1/21.
//
#ifndef ROS2MASTER_PID_H
#define ROS2MASTER_PID_H

#include <algorithm>
#include <cmath>

namespace nif {
    namespace utils {
        /// @class PID
        /// @brief A generic PID controller class that uses a discrete approximation
        /// of a continuous PID controller.
        class PID {
        public:
            /// @brief Constructor
            /// @param[in] p is the proportional gain. Defaults to 0.0.
            /// @param[in] i is the integral gain. Defaults to 0.0.
            /// @param[in] d is the derivative gain. Defaults to 0.0.
            /// @param[in] iMax is a bound on the maximum integrated error (+/-). Defaults
            /// to 1e8.
            /// @param[in] cmdMax is an upper bound on the output command. Defaults to
            /// 1e8.
            /// @param[in] cmdMin is a lower bound on the output command. Defaults to
            /// -1e8.
            PID(const double &p = 0.0, const double &i = 0.0, const double &d = 0.0,
                const double &dt = 0.01, const double &iMax = 1e8,
                const double &cmdMax = 1e8, const double &cmdMin = -1e8);

            /// @brief Get the current proportional gain.
            /// @return The current value of the proportional gain.
            inline double P() const { return p_; }

            /// @brief Get the current integral gain.
            /// @return The current value of the integral gain.
            inline double I() const { return i_; }

            /// @brief Get the current derivative gain.
            /// @return The current derivative gain.
            inline double D() const { return d_; }

            /// @brief Get the controller time step
            /// @return the current value of the member dt_
            inline double TimeStep() const { return dt_; }

            /// @brief Get the value of the iMax_ member
            /// @return value of iMax_
            inline double IMax() const { return iMax_; }

            /// @brief Get the value of the cmdMax_ member
            /// @return value of cmdMax_
            inline double CmdMax() const { return cmdMax_; }

            /// @brief Get the value of the cmdMin_ member
            /// @return value of cmdMin_
            inline double CmdMin() const { return cmdMin_; }

            /// @brief Get the current value of the control error
            inline double CurrentError() const { return e_; }

            /// @brief Get the previous value of the control error
            inline double PreviousError() const { return eOld_; }

            /// @brief Get the value of the integrated control error
            inline double ErrorIntegral() const { return eInt_; }

            /// @brief Reset the value of the integrated control error to zero
            inline void ResetErrorIntegral() { this->eInt_ = 0.0; }

            /// @brief Input the current error and get back the current control value
            /// @param newError is the most recently calculated error value
            /// @return the control value from the PID algorithm
            double Update(const double &newError);

            /// @brief Get the current control value (calculated based on most recent
            /// errors)
            /// @return the current control value
            double CurrentControl();

            /// @brief Set the value of the proportional gain
            /// @param p is a new proportional gain. Must be greater than zero.
            /// @return true if set was successful. false otherwise.
            bool SetPGain(const double &p);

            /// @brief Set the value of the integral gain
            /// @param i is the new integral gain. Must be greater than zero.
            /// @return true if set was successful. false otherwise.
            bool SetIGain(const double &i);

            /// @brief Set the value of the derivative gain
            /// @param d is the new derivative gain. Must be greater than zero.
            /// @return true if set was successful. false otherwise.
            bool SetDGain(const double &d);

            /// @brief Set the time step used by the controller to integrate error
            /// @param dt is the new time step to use. Must be greater than zero.
            /// @return true if set was successful. false otherwise.
            bool SetTimeStep(const double &dt);

            /// @brief Set the maximum value error accumulation value
            /// @param iMax is the new max value of the integrated control error
            void SetIMax(const double &iMax);

            /// @brief Set the minimum and maximum bounds on the command output
            /// @param min is the minimum command output. Must be less than or equal to
            /// zero.
            /// @param max is the maximum command output. Must be greater than zero.
            /// @return a pair of bools. First is true if min is set correctly, second is
            /// true if max is set correctly.
            std::pair<bool, bool> SetCmdBounds(const double &min, const double &max);

        private:
            /// @brief Saturate the command value based on the max and min limits
            double SaturateCmd(const double &cmd);

            /// @brief Saturate the value based on iMax_
            void SaturateIntegratedError();

            /// @brief Proportional gain.
            double p_ = 0.0;
            /// @brief Integral gain
            double i_ = 0.0;
            /// @brief Derivative gain
            double d_ = 0.0;

            /// @brief The time step the controller is running at. Must be greater than
            /// zero.
            double dt_ = 0.01;

            /// @brief Maximum integral error (+/-)
            double iMax_ = 1e8;
            /// @brief Maximum output command. Must be greater than zero.
            double cmdMax_ = 1e8;
            /// @brief Minimum output command. Must be less than or equal to zero.
            double cmdMin_ = -1e8;

            /// @brief Current control error used in PID control algorithm
            double e_ = 0.0;
            /// @brief Previous control error used in PID control algorithm
            double eOld_ = 0.0;
            /// @brief Current integrated value of the control error used in the PID
            /// control algorithm
            double eInt_ = 0.0;
        };  // End class PID
    }
}

#endif //ROS2MASTER_PID_H
