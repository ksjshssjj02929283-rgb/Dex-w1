#pragma once

namespace robot_server_lib::dynamics::w1_v03 {
    void left_mass_term_func(double* M_term_out, const double* q, const double* ddq, const double* dyn_base_params);
    void left_regression_func(double* tau_out, const double* q, const double* dq, const double* ddq, const double* dyn_base_params, const double* gravity_acc);
    void right_mass_term_func(double* M_term_out, const double* q, const double* ddq, const double* dyn_base_params);
    void right_regression_func(double* tau_out, const double* q, const double* dq, const double* ddq, const double* dyn_base_params, const double* gravity_acc);
}
