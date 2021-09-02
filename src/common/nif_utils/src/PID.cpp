#include "nif_utils/PID.hpp"

namespace nif {
    namespace utils {

        PID::PID(const double &p, const double &i, const double &d, const double &dt,
                 const double &iMax, const double &cmdMax, const double &cmdMin) {
            // Set valid gains
            SetPGain(p);
            SetIGain(i);
            SetDGain(d);

            // Set time step
            SetTimeStep(dt);

            // Set valid bounds
            this->iMax_ = std::abs(iMax);
            SetCmdBounds(cmdMin, cmdMax);
        }

/****************************** PID ALGORITHM *******************************/
        double PID::Update(const double &newError) {
            this->eOld_ = e_;
            this->e_ = newError;
            this->eInt_ += (e_ - eOld_);
            SaturateIntegratedError();
            return CurrentControl();
        }

        double PID::CurrentControl() {
            double dError = (e_ - eOld_) / dt_;
            double cmd = p_ * e_ + i_ * eInt_ + d_ * dError;
            return SaturateCmd(cmd);
        }

/******************************* SETTERS ************************************/
        bool PID::SetPGain(const double &p) {
            if (p > 0.0) {
                this->p_ = p;
                return true;
            }
            return false;
        }

        bool PID::SetIGain(const double &i) {
            if (i > 0.0) {
                this->i_ = i;
                return true;
            }
            return false;
        }

        bool PID::SetDGain(const double &d) {
            if (d > 0.0) {
                this->d_ = d;
                return true;
            }
            return false;
        }

        bool PID::SetTimeStep(const double &dt) {
            if (dt > 0.0) {
                this->dt_ = dt;
                return true;
            }
            return false;
        }

        void PID::SetIMax(const double &iMax) { this->iMax_ = std::abs(iMax); }

        std::pair<bool, bool> PID::SetCmdBounds(const double &min, const double &max) {
            bool isMinSet = false, isMaxSet = false;
            if (min <= 0.0) {
                this->cmdMin_ = min;
                isMinSet = true;
            }
            if (max > 0.0) {
                this->cmdMax_ = max;
                isMaxSet = true;
            }
            return std::make_pair(isMinSet, isMaxSet);
        }

/*********************** PRIVATE METHODS ************************************/
        double PID::SaturateCmd(const double &cmd) {
            return std::min(std::max(cmdMin_, cmd), cmdMax_);
        }

        void PID::SaturateIntegratedError() {
            this->eInt_ = std::min(std::max(-iMax_, eInt_), iMax_);
        }

    }
}