#include <fw_nmpc/nonlinear_mpc_objectives.h>
#include <fw_nmpc/common/huber_constraint.h>
#include <math.h>

namespace fw_nmpc {

/*
    AIRSPEED OBJECTIVE
*/

void NonlinearMPCObjectives::jacobianAirsp(double *jac)
{
    /*
        Jacobian of airspeed w.r.t.:
        airspeed

        Outputs:
        jac[1]
    */

    jac[0] = 1.0;
} // jacobianAirsp

/*
    END AIRSPEED OBJECTIVE
*/

/*
    ANGLE OF ATTACK (AOA) OBJECTIVE
*/

void NonlinearMPCObjectives::jacobianAOA(double *jac)
{
    /*
        Jacobian of AoA w.r.t.:
        pitch angle
        flight path angle

        Outputs:
        jac[2]
    */

    jac[0] = -1.0;
    jac[1] = 1.0;
} // jacobianAOA

/*
    END ANGLE OF ATTACK (AOA) OBJECTIVE
*/

/*
    HEIGHT ABOVE GROUND LEVEL (HAGL) OBJECTIVE
*/

void NonlinearMPCObjectives::jacobianHAGL(double *jac,
    double *corners, const double h12, const double h34, const double de, const double terr_dis)
{
    /*
        Jacobian of HAGL w.r.t.:
        position north
        position east
        position down
        heading XXX: why is this here? -> use to have an offset height probe option which then depended on heading..

        Inputs:
        TODO: more descriptive variable names

        Outputs:
        jac[4]
    */

    const double inv_terr_dis = 1.0/terr_dis;
    jac[0] = (de * (corners[2] - corners[3]) + (1.0 - de) * (corners[0] - corners[1])) * inv_terr_dis;
    jac[1] = (h12 - h34) * inv_terr_dis;
    jac[2] = -1.0;
    jac[3] = 0.0;
} // jacobianHAGL

/*
    END HEIGHT ABOVE GROUND LEVEL (HAGL) OBJECTIVE
*/

/*
    RADIAL TERRAIN DISTANCE (RTD) OBJECTIVE
*/

double NonlinearMPCObjectives::calculateRTDConstraint(const double rel_ground_sp_sq, const double rtd_constr_0, const double rtd_constr_scaler)
{
    /*
        Radial Terrain Distance (RTD) Constraint

        inputs:
        rel_ground_sp_sq    ground speed relative to the occlusion squared [m^2/s^2]
        rtd_constr_0        the constraint value at ground speed = 0m/s [m]
        rtd_constr_scaler   = rtd_constr_gain/g/tan(roll_max) [s^2/m]

        returns:
        rtd constraint value [m]
    */
    return rtd_constr_0 + rel_ground_sp_sq * rtd_constr_scaler;
} // calculateRTDConstraint

double NonlinearMPCObjectives::calculateRTDDelta(const double rel_ground_sp_sq, const double rtd_delta_0, const double rtd_delta_scaler)
{
    /*
        Radial Terrain Distance (RTD) Delta

        inputs:
        rel_ground_sp_sq   ground speed relative to the occlusion squared [m^2/s^2]
        rtd_delta_0        the delta value at ground speed = 0m/s [m]
        rtd_delta_scaler   = rtd_delta_gain/g/tan(roll_max) [s^2/m]

        returns:
        rtd delta value [m]
    */
    return rtd_delta_0 + rel_ground_sp_sq * rtd_delta_scaler;
} // calculateRTDDelta

void NonlinearMPCObjectives::calculateRTDCostAndJacobian(double &rtd_cost, double *rtd_jac, const int len_jac, double &rtd_inv_prio,
    const Eigen::Vector3d &occ_pos, const Eigen::Vector3d &occ_normal, const Eigen::Vector3d &veh_pos,
    const Eigen::Vector3d &ground_vel, const Eigen::Matrix<double, 3, 3> &J_vel, const double surfel_radius,
    const double rtd_constr_0, const double rtd_constr_scaler, const double rtd_delta_0, const double rtd_delta_scaler)
{
    /*
        Radial Terrain Distance (RTD) huber cost, jacobian, and inverse priority

        inputs:
        TODO

        outputs:
        rtd_cost        huber cost
        rtd_jac         huber jacobian
        rtd_inv_prio    inverse priority
    */

    // distance vector ("ray") to occlusion
    const Eigen::Vector3d ray = occ_pos - veh_pos;

    // dot product of ray and occlusion normal
    const double dot_ray_occ_normal = ray.dot(occ_normal);

    if (dot_ray_occ_normal < 0.0) {
        // only consider when in front of occlusion

        // (closest) distance to occlusion surface, "terrain distance" (projection of ray onto normal vector)
        Eigen::Vector3d td = dot_ray_occ_normal * occ_normal;

        // projection of ray onto surface plane
        const Eigen::Vector3d ray_parallel = ray - td;
        const double norm_ray_parallel = ray_parallel.norm();

        double norm_td = 0.0;
        Eigen::Vector3d unit_td(0.0,0.0,0.0);
        double jac_td[len_jac];
        double jac_rel_vel[len_jac];
        // check surfel radius
        if (norm_ray_parallel > surfel_radius || surfel_radius < MIN_SURFEL_RADIUS) {
            // treat edge of surfel as a point, get jacobian w.r.t. said point

            // recalculate terrain distance to new point
            if (norm_ray_parallel > 0.0) {
                td = ray - surfel_radius * ray_parallel / norm_ray_parallel;
            }
            else {
                td = ray;
            }

            // unit distance vector
            norm_td = td.norm();
            if (norm_td > 0.0) {
                unit_td = td / norm_td;
            }

            // jacobian of the terrain distance to a point
            jacobianTDToPoint(jac_td, unit_td);

            // jacobian of the relative velocity to a point
            jacobianRelativeVelocityToPoint(jac_rel_vel, unit_td, norm_td, ground_vel, J_vel);
        }
        else {
            // get the jacobian with respect to the surface

            // unit distance vector
            norm_td = td.norm();
            if (norm_td > 0.0) {
                unit_td = td / norm_td;
            }

            // jacobian of the terrain distance to the occlusion surface
            jacobianTDToSurface(jac_td, occ_normal);

            // jacobian of the relative velocity to a surface
            jacobianRelativeVelocityToSurface(jac_rel_vel, occ_normal, J_vel);
        }

        // relative speed
        double rel_ground_sp = unit_td.dot(ground_vel);
        if (rel_ground_sp < 0.0) {
            // no negative speeds when flying away from surface
            rel_ground_sp = 0.0;
        }

        // partial jacobians
        double d_rtd_d_constr[len_jac];
        jacobianRTDConstraint(d_rtd_d_constr, jac_rel_vel, rel_ground_sp, rtd_constr_scaler);
        double d_rtd_d_delta[len_jac];
        jacobianRTDDelta(d_rtd_d_delta, jac_rel_vel, rel_ground_sp, rtd_delta_scaler);

        // calculate the huber cost and jacobian XXX: is it expensive to construct this every loop?
        HuberConstraint huber_rtd;
        huber_rtd.setConstraint(calculateRTDConstraint(rel_ground_sp * rel_ground_sp, rtd_constr_0, rtd_constr_scaler));
        huber_rtd.setDelta(calculateRTDDelta(rel_ground_sp * rel_ground_sp, rtd_delta_0, rtd_delta_scaler));
        huber_rtd.costAndJacobian(rtd_cost, rtd_jac, rtd_inv_prio, norm_td, jac_td, len_jac, d_rtd_d_constr, d_rtd_d_delta);
    }
} // calculateRTDCostAndJacobian

void NonlinearMPCObjectives::calculateVelocityJacobianMatrix(Eigen::Matrix<double, 3, 3> &J_vel,
    const double airspeed, const double fpa, const double heading)
{
    const double cos_fpa = cosf(fpa);
    const double sin_fpa = sinf(fpa);
    const double cos_heading = cosf(heading);
    const double sin_heading = sinf(heading);

    J_vel(0,0) = cos_fpa * cos_heading;
    J_vel(0,1) = cos_fpa * sin_heading;
    J_vel(0,2) = -sin_fpa;
    J_vel(1,0) = -airspeed * sin_fpa * cos_heading;
    J_vel(1,1) = -airspeed * sin_fpa * sin_heading;
    J_vel(1,2) = -airspeed * cos_fpa;
    J_vel(2,0) = -airspeed * cos_fpa * sin_heading;
    J_vel(2,1) = airspeed * cos_fpa * cos_heading;
    J_vel(2,2) = 0;
} // calculateVelocityJacobianMatrix

void NonlinearMPCObjectives::jacobianRelativeVelocityToPoint(double *jac_rel_vel,
    const Eigen::Vector3d &unit_td, const double norm_td, const Eigen::Vector3d &ground_vel,
    const Eigen::Matrix<double, 3, 3> &J_vel)
{
    /*
        Jacobian of relative velocity to a point w.r.t.:
        position north
        position east
        position down
        airspeed
        flight path angle
        heading
    */

    double inv_td = 0.0;
    if (norm_td > 0.1) inv_td = 1.0 / norm_td;

    // gradient vrel w.r.t. position
    Eigen::Vector3d grad_vrel_position = (ground_vel.dot(unit_td) * unit_td - ground_vel) * inv_td;

    // gradient vrel w.r.t. polar airspeed vector (v, gamma, xi)
    Eigen::Vector3d grad_vrel_polar_airsp = J_vel * unit_td;

    jac_rel_vel[0] = grad_vrel_position(0);
    jac_rel_vel[1] = grad_vrel_position(1);
    jac_rel_vel[2] = grad_vrel_position(2);
    jac_rel_vel[3] = grad_vrel_polar_airsp(0);
    jac_rel_vel[4] = grad_vrel_polar_airsp(1);
    jac_rel_vel[5] = grad_vrel_polar_airsp(2);
} // jacobianRelativeVelocityToPoint

void NonlinearMPCObjectives::jacobianRelativeVelocityToSurface(double *jac_rel_vel,
    const Eigen::Vector3d &occ_normal, const Eigen::Matrix<double, 3, 3> &J_vel)
{
    /*
        Jacobian of relative velocity to a surface w.r.t.:
        position north
        position east
        position down
        airspeed
        flight path angle
        heading
    */

    // gradient vrel w.r.t. polar airspeed vector (v, gamma, xi)
    Eigen::Vector3d grad_vrel_polar_airsp = -J_vel * occ_normal;

    jac_rel_vel[0] = 0.0;
    jac_rel_vel[1] = 0.0;
    jac_rel_vel[2] = 0.0;
    jac_rel_vel[3] = grad_vrel_polar_airsp(0);
    jac_rel_vel[4] = grad_vrel_polar_airsp(1);
    jac_rel_vel[5] = grad_vrel_polar_airsp(2);
} // jacobianRelativeVelocityToSurface

void NonlinearMPCObjectives::jacobianRTDConstraint(double *jac,
    double *jac_vrel, const double rel_ground_sp, const double rtd_constr_scaler)
{
    /*
        Jacobian of RTD constraint w.r.t.:
        position north
        position east
        position down
        airspeed
        flight path angle
        heading

        Inputs:
        jac_vrel[6]
        rel_ground_sp
        rtd_constr_scaler

        Outputs:
        jac[6]
    */

    const double t = 2.0 * rel_ground_sp * rtd_constr_scaler;
    jac[0] = t * jac_vrel[0];
    jac[1] = t * jac_vrel[1];
    jac[2] = t * jac_vrel[2];
    jac[3] = t * jac_vrel[3];
    jac[4] = t * jac_vrel[4];
    jac[5] = t * jac_vrel[5];
} // jacobianRTDConstraint

void NonlinearMPCObjectives::jacobianRTDDelta(double *jac,
    double *jac_vrel, const double rel_ground_sp, const double rtd_delta_scaler)
{
    /*
        Jacobian of RTD delta w.r.t.:
        position north
        position east
        position down
        airspeed
        flight path angle
        heading

        Inputs:
        jac_vrel[6]
        rel_ground_sp
        rtd_delta_scaler

        Outputs:
        jac[6]
    */

    const double t = 2.0 * rel_ground_sp * rtd_delta_scaler;
    jac[0] = t * jac_vrel[0];
    jac[1] = t * jac_vrel[1];
    jac[2] = t * jac_vrel[2];
    jac[3] = t * jac_vrel[3];
    jac[4] = t * jac_vrel[4];
    jac[5] = t * jac_vrel[5];
} // jacobianRTDDelta

void NonlinearMPCObjectives::jacobianTDToPoint(double *jac, const Eigen::Vector3d &unit_td)
{
    jac[0] = -unit_td(0);
    jac[1] = -unit_td(1);
    jac[2] = -unit_td(2);
    jac[3] = 0.0;
    jac[4] = 0.0;
    jac[5] = 0.0;
} // jacobianTDToPoint

void NonlinearMPCObjectives::jacobianTDToSurface(double *jac, const Eigen::Vector3d &occ_normal)
{
    jac[0] = occ_normal(0);
    jac[1] = occ_normal(1);
    jac[2] = occ_normal(2);
    jac[3] = 0.0;
    jac[4] = 0.0;
    jac[5] = 0.0;
} // jacobianTDToSurface

/*
    END RADIAL TERRAIN DISTANCE (RTD) OBJECTIVE
*/

} // namespace fw_nmpc
