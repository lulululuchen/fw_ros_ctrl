#ifndef PWQG_H_
#define PWQG_H_

#include <Eigen/Eigen>
#include <math.h>

namespace fw_nmpc {

/*
    PWQG class
    Longitudinal Piecewise Quadratic Guidance Logic
 */
class PWQG {

    public:

        PWQG();

        // sets
        void setFixedVertPosErrBound(const bool en) { fix_vert_pos_err_bnd_ = en; }
        void setTimeConstant(const double tc) { time_const_ = tc; }
        void setVertPosErrBound(const double bnd) { vert_pos_err_bnd_ = std::max(bnd, 0.1); }
        void setFPAAppMax(const double fpa) { fpa_app_max_ = std::max(fpa, 0.01); }

        // gets
        double getFPAAppMax() { return fpa_app_max_; }

        // public functions
        double evaluate(double &err_lon, double *jac_fpa_sp, const double d_ref, const double unit_path_tangent_d, const double veh_pos_d,
            const double ground_speed, const double airspeed, const double wind_vel_d);

    private:

        bool fix_vert_pos_err_bnd_;
        double time_const_;
        double fpa_app_max_;
        double vert_pos_err_bnd_;

        // private functions
        double calcTrackErrorBound(const double ground_speed);
}; // class PWQG

} // namespace fw_nmpc

#endif // PWQG_H_
