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

        enum CarrotDynamics {
            LINEAR = 0,
            QUADRATIC
        };

        // sets
        void setFixedVertPosErrBound(const bool en) { fix_vert_pos_err_bnd_ = en; }
        void setTimeConstant(const double tc) { time_const_ = tc; }
        void setVertPosErrBound(const double bnd) { vert_pos_err_bnd_ = std::max(bnd, 0.1); }
        void setFPAClimb(const double fpa) { fpa_climb_ = fpa; }; // TODO: need to constrain what sink / climb are allowed to be
        void setFPASink(const double fpa) { fpa_sink_ = fpa; };
        void setCarrotScale(const double sc) { pos_carrot_scale_ = sc; }
        void setCarrotDynamics(CarrotDynamics dyn) { pos_carrot_dyn_ = dyn; }
        void setMaxUnitErrLon(double max) { max_unit_err_lon_ = max; }

        // gets
        double getFPAClimb() { return fpa_climb_; };
        double getFPASink() { return fpa_sink_; };

        // public functions
        double evaluate(double &err_lon, double *jac_fpa_sp, bool &feas, const double d_ref, const double unit_path_tangent_d, const double veh_pos_d,
            const double ground_speed, const double airspeed, const double wind_vel_d, const Eigen::Vector3d &ground_vel, const Eigen::Vector3d &air_vel);
        double calcPositionCarrot(const double veh_pos_d, const double d_ref, const double ground_speed);

    private:

        bool fix_vert_pos_err_bnd_;
        double time_const_;
        double fpa_climb_;
        double fpa_sink_;
        double vert_pos_err_bnd_;

        double pos_carrot_scale_;
        CarrotDynamics pos_carrot_dyn_;

        double max_unit_err_lon_;

        // private functions
        double calcTrackErrorBound(const double ground_speed);
}; // class PWQG

} // namespace fw_nmpc

#endif // PWQG_H_
