#ifndef NONLINEAR_MPC_OBJECTIVES_H_
#define NONLINEAR_MPC_OBJECTIVES_H_

#include <Eigen/Eigen>

namespace fw_nmpc {

/*
    NonlinearMPCObjectives class
    Library of functions for external evaluation of NMPC objectives
 */
class NonlinearMPCObjectives {

    public:

        NonlinearMPCObjectives() {}

        static constexpr double MIN_SURFEL_RADIUS = 0.1;   // minimum surfel radius for occlusions (below this implies we consider only points) [m]

        /* Angle of Attack (AoA) Objective */
        void jacobianAOA(double *jac);

        /* Height Above Ground Level (HAGL) Objective */
        void jacobianHAGL(double *jac, double *corners, const double h12, const double h34, const double de, const double terr_dis);

        /* Radial Terrain Distance (RTD) Objective */
        double calculateRTDConstraint(const double rel_ground_sp_sq, const double rtd_constr_0, const double rtd_constr_scaler);
        double calculateRTDDelta(const double rel_ground_sp_sq, const double rtd_delta_0, const double rtd_delta_scaler);
        void calculateRTDCostAndJacobian(double &rtd_cost, double *rtd_jac, const int len_jac, double &rtd_inv_prio,
            const Eigen::Vector3d &occ_pos, const Eigen::Vector3d &occ_normal, const Eigen::Vector3d &veh_pos,
            const Eigen::Vector3d &ground_vel, const Eigen::Matrix<double, 3, 3> &J_vel, const double surfel_radius,
            const double rtd_constr_0, const double rtdconstr_scaler, const double rtd_delta_0, const double rtd_delta_scaler);
        void calculateVelocityJacobianMatrix(Eigen::Matrix<double, 3, 3> &J_vel, const double airspeed, const double fpa, const double heading);
        void jacobianRelativeVelocityToPoint(double *jac_rel_vel, const Eigen::Vector3d &unit_td, const double norm_td,
            const Eigen::Vector3d &ground_vel, const Eigen::Matrix<double, 3, 3> &J_vel);
        void jacobianRelativeVelocityToSurface(double *jac_rel_vel, const Eigen::Vector3d &occ_normal, const Eigen::Matrix<double, 3, 3> &J_vel);
        void jacobianRTDConstraint(double *jac, double *jac_vrel, const double rel_ground_sp, const double rtd_constr_scaler);
        void jacobianRTDDelta(double *jac, double *jac_vrel, const double rel_ground_sp, const double rtd_constr_scaler);
        void jacobianTDToPoint(double *jac, const Eigen::Vector3d &unit_td);
        void jacobianTDToSurface(double *jac, const Eigen::Vector3d &occ_normal);
}; // class NonlinearMPCObjectives

} // namespace fw_nmpc

#endif // NONLINEAR_MPC_OBJECTIVES_H_
