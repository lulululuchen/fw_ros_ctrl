#include <fw_nmpc/common/huber_constraint.h>
#include <math.h>

namespace fw_nmpc {

/*
    HUBER CONSTRAINT
*/

HuberConstraint::HuberConstraint(const std::string &constr_type) :
    constraint_(1.0),
    delta_(1.0),
    sign_(double(1-2*(constr_type=="upper")))
{} // HuberConstraint

double HuberConstraint::normalizeState(const double state)
{
    return sign_ * (state - constraint_) / delta_;
} // normalizeState

double HuberConstraint::inversePriority(const double normalized_state)
{
    return (normalized_state < 0.0) ? 0.0 : ((normalized_state > 1.0) ? 1.0 : normalized_state);
} // inversePriority

double HuberConstraint::chainJacobians(const double normalized_state, const double state_jac, const double constr_jac, const double delta_jac)
{
    /*
        applies chain rule to obtain jacobian of normalized state from provided state, constraint, and delta jacobians
    */
    return sign_ * (state_jac - constr_jac - normalized_state * delta_jac) / delta_;
} // chainJacobians

void HuberConstraint::costAndJacobian(double &huber_cost, double *huber_jac, double &inv_prio,
    const double state, double *state_jac, int len_jac, double *constr_jac, double *delta_jac)
{
    /*
        calculates and returns Huber Constraint and Jacobian (w.r.t. state dependencies), also the "inv_priority"

        NOTE: all state, constraint, and delta jacobians must be calculated externally and provided.
              if using constraint or delta jacobians, remember to also "set" the current constraint
              and delta before calling this function.

        inputs:
        state               current (un-normalized) state
        state_jac[len_jac]  state jacobian: d(state)/d(state dependencies)
        len_jac             length of jacobian (num of state dependencies)

        <optional> inputs:
        constr_jac[len_jac] constraint jacobian: d(constraint)/d(state dependencies)
        delta_jac[len_jac]  delta jacobian: d(delta)/d(state dependencies)

        outputs:
        huber_cost          Huber cost
        huber_jac[len_jac]  Huber cost jacobian:
                            d(Huber loss)/d(state) * d(state)/d(state dependencies)
        inv_prio            inverse "priority" of the soft constraint (can be multiplied by the other objectives directly for prioritization)
    */

    const double normalized_state = normalizeState(state);

    if (normalized_state < 0.0) {
        // linear

        // cost
        huber_cost = 1.0 - 2.0 * normalized_state;

        // jacobian //TODO: there is probably a cleaner / better way to do this
        if (delta_jac) {
            // both delta and constraint additionally have state dependencies
            for (int i = 0; i < len_jac; i++) {
                huber_jac[i] = -2.0 * chainJacobians(normalized_state, state_jac[i], constr_jac[i], delta_jac[i]);
            }
        }
        else if (constr_jac) {
            // only the constraint additionally has state dependencies
            for (int i = 0; i < len_jac; i++) {
                huber_jac[i] = -2.0 * chainJacobians(normalized_state, state_jac[i], constr_jac[i]);
            }
        }
        else {
            // no delta or constraint dependencies
            for (int i = 0; i < len_jac; i++) {
                huber_jac[i] = -2.0 * chainJacobians(normalized_state, state_jac[i]);
            }
        }
    }
    else if (normalized_state < 1.0) {
        // quadratic

        // cost
        const double sqrt_huber_cost = 1.0 - normalized_state;
        huber_cost = sqrt_huber_cost * sqrt_huber_cost;

        // jacobian //TODO: there is probably a cleaner / better way to do this
        if (delta_jac) {
            // both delta and constraint additionally have state dependencies
            for (int i = 0; i < len_jac; i++) {
                huber_jac[i] = -2.0 * sqrt_huber_cost * chainJacobians(normalized_state, state_jac[i], constr_jac[i], delta_jac[i]);
            }
        }
        else if (constr_jac) {
            // only the constraint additionally has state dependencies
            for (int i = 0; i < len_jac; i++) {
                huber_jac[i] = -2.0 * sqrt_huber_cost * chainJacobians(normalized_state, state_jac[i], constr_jac[i]);
            }
        }
        else {
            // no delta or constraint dependencies
            for (int i = 0; i < len_jac; i++) {
                huber_jac[i] = -2.0 * sqrt_huber_cost * chainJacobians(normalized_state, state_jac[i]);
            }
        }
    }
    else {
        // zero

        huber_cost = 0.0;

        for (int i = 0; i < len_jac; i++) {
            huber_jac[i] = 0.0;
        }
    }

    // inv_priority
    inv_prio = inversePriority(normalized_state);

} // CostAndJacobian

/*
    END HUBER CONSTRAINT
*/

/*
    EXPONENTIAL HUBER CONSTRAINT
*/

ExponentialHuberConstraint::ExponentialHuberConstraint(const std::string &constr_type) :
    HuberConstraint(constr_type),
    enabled_(false),
    scale_(1.0),
    cost_at_one_(0.001)
{} // ExponentialHuberConstraint

void ExponentialHuberConstraint::updateScale(const double lsq_weight)
{
    if (lsq_weight < cost_at_one_) {
        // disable loss calculations
        enabled_ = false;
    }
    else {
        // reset enable switch
        if (!enabled_) enabled_ = true;

        // update scale
        scale_ = log(sqrt(lsq_weight / cost_at_one_));
        // NOTE: sqrt here is necessary as we are actually calculating the output that will eventually be squared in the least squares objective (ACADO does not allow custom objective forms)
    }
} // updateScale

void ExponentialHuberConstraint::costAndJacobian(double &huber_cost, double *huber_jac, double &inv_prio,
    const double state, double *state_jac, int len_jac, double *constr_jac, double *delta_jac)
{
    /*
        calculates and returns exponential Huber constraint and Jacobian (w.r.t. state dependencies), also the "inv_priority"

        NOTE: all state, constraint, and delta jacobians must be calculated externally and provided.
              if using constraint or delta jacobians, remember to also "set" the current constraint
              and delta before calling this function.

        inputs:
        state               current (un-normalized) state
        state_jac[len_jac]  state jacobian: d(state)/d(state dependencies)
        len_jac             length of jacobian (num of state dependencies)

        <optional> inputs:
        constr_jac[len_jac] constraint jacobian: d(constraint)/d(state dependencies)
        delta_jac[len_jac]  delta jacobian: d(delta)/d(state dependencies)

        outputs:
        huber_cost          Huber cost
        huber_jac[len_jac]  Huber cost jacobian:
                            d(Huber loss)/d(state) * d(state)/d(state dependencies)
        inv_prio            inverse "priority" of the soft constraint (can be multiplied by the other objectives directly for prioritization)
    */

    if (enabled_) {

        const double normalized_state = normalizeState(state);

        if (normalized_state < 0.0) {
            // linear

            // cost
            huber_cost = 1.0 - normalized_state * scale_;

            // jacobian //TODO: there is probably a cleaner / better way to do this
            if (delta_jac) {
                // both delta and constraint additionally have state dependencies
                for (int i = 0; i < len_jac; i++) {
                    huber_jac[i] = -scale_ * chainJacobians(normalized_state, state_jac[i], constr_jac[i], delta_jac[i]);
                }
            }
            else if (constr_jac) {
                // only the constraint additionally has state dependencies
                for (int i = 0; i < len_jac; i++) {
                    huber_jac[i] = -scale_ * chainJacobians(normalized_state, state_jac[i], constr_jac[i]);
                }
            }
            else {
                // no delta or constraint dependencies
                for (int i = 0; i < len_jac; i++) {
                    huber_jac[i] = -scale_ * chainJacobians(normalized_state, state_jac[i]);
                }
            }
        }
        else {
            // exponential

            // cost
            huber_cost = exp(-normalized_state * scale_);

            // jacobian //TODO: there is probably a cleaner / better way to do this
            if (delta_jac) {
                // both delta and constraint additionally have state dependencies
                for (int i = 0; i < len_jac; i++) {
                    huber_jac[i] = -scale_ * huber_cost * chainJacobians(normalized_state, state_jac[i], constr_jac[i], delta_jac[i]);
                }
            }
            else if (constr_jac) {
                // only the constraint additionally has state dependencies
                for (int i = 0; i < len_jac; i++) {
                    huber_jac[i] = -scale_ * huber_cost * chainJacobians(normalized_state, state_jac[i], constr_jac[i]);
                }
            }
            else {
                // no delta or constraint dependencies
                for (int i = 0; i < len_jac; i++) {
                    huber_jac[i] = -scale_ * huber_cost * chainJacobians(normalized_state, state_jac[i]);
                }
            }
        }

        // inv_priority
        inv_prio = inversePriority(normalized_state);
    }
    else {
        // disabled, zero all

        huber_cost = 0.0;

        for (int i = 0; i < len_jac; i++) {
            huber_jac[i] = 0.0;
        }

        inv_prio = 1.0;
    }
} // costAndJacobian

/*
    END EXPONENTIAL HUBER CONSTRAINT
*/

} // namespace fw_nmpc
