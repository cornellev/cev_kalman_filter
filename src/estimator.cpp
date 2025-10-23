#include "estimator.h"

using namespace ckf;

Estimator::Estimator(V state, M covariance)
    : state(state), covariance(covariance) {
}

V Estimator::get_state() const {
    return state;
}

M Estimator::get_covariance() const {
    return covariance;
}

StatePackage Estimator::get_internals() const {
    return {state, covariance, most_recent_update_time};
}

void Estimator::updateInternals(const StatePackage& package) {
    state = package.state;
    covariance = package.covariance;
    previous_update_time = most_recent_update_time;
    most_recent_update_time = package.update_time;
}

void Estimator::updateInternals(const SimpleStatePackage& package) {
    state = package.state;
    previous_update_time = most_recent_update_time;
    most_recent_update_time = package.update_time;
}

double Estimator::get_most_recent_update_time() {
    return most_recent_update_time;
}

double Estimator::get_previous_update_time() {
    return previous_update_time;
}

double Estimator::dt() {
    return most_recent_update_time - previous_update_time;
}

M Estimator::state_mask_to_matrix(const std::vector<std::string>& state_mask) {
    M mask_matrix = M::Zero(S, S);
    for (const std::string& state_name: state_mask) {
        int state_index = ckf::state::string_state.at(state_name);
        mask_matrix(state_index, state_index) = 1;
    }
    return mask_matrix;
}
