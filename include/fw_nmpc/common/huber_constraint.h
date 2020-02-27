#ifndef HUBER_CONSTRAINT_H_
#define HUBER_CONSTRAINT_H_

#include <string>

namespace fw_nmpc {

/*
    HuberConstraint class
    Implementation of a one-sided Huber loss, used for soft NMPC constraints
*/
class HuberConstraint {
    public:
        HuberConstraint(const std::string &constr_type = "lower");

        // sets
        void setConstraint(const double constraint) { constraint_ = constraint; }
        void setDelta(const double delta) { delta_ = (delta < MIN_DELTA) ? MIN_DELTA : delta; }

        // virtual functions
        virtual void costAndJacobian(double &huber_cost, double *huber_jac, double &inv_prio,
            const double state, double *state_jac, int len_jac, double *constr_jac = nullptr, double *delta_jac = nullptr);

    protected:

        // functions
        double chainJacobians(const double normalized_state, const double state_jac, const double constr_jac = DOUBLE_ZERO, const double delta_jac = DOUBLE_ZERO);
        double inversePriority(const double normalized_state);
        double normalizeState(const double state);

    private:

        // constants
        static constexpr double MIN_DELTA = 1.0e-5; // must be > 0
        static constexpr double DOUBLE_ZERO = 0.0;

        // variables
        double constraint_;
        double delta_;
        double sign_;
}; // class HuberConstraint

/*
    Exponential HuberConstraint class (derived from HuberConstraint)
    Exponential implementation of a one-sided Huber loss, used for soft NMPC constraints
*/
class ExponentialHuberConstraint : public HuberConstraint //TODO: should encapsulate individual cost/jacobians smarter and only virtualize them instead of the whole chaining logic
{
    public:
        ExponentialHuberConstraint(const std::string &constr_type = "lower");

        // sets
        void setCostAtOne(const double cost) { cost_at_one_ = (cost < MIN_COST_AT_ONE) ? MIN_COST_AT_ONE : cost; }

        // functions
        void costAndJacobian(double &huber_cost, double *huber_jac, double &inv_prio,
            const double state, double *state_jac, int len_jac, double *constr_jac = nullptr, double *delta_jac = nullptr) override;
        void updateScale(const double lsq_weight);

    private:

        // constants
        static constexpr double MIN_COST_AT_ONE = 1.0e-5; // minimum exponential cost at normalized state = 1 (for weight = 1)

        // variables
        bool enabled_;
        double scale_;
        double cost_at_one_;
}; // class ExponentialHuberConstraint

} // namespace fw_nmpc

#endif // HUBER_CONSTRAINT_H_
