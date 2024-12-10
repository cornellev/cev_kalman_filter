#include "estimator.h"

using namespace ckf;

Estimator::Estimator(V state, M covariance) {
    this->state = state;
    this->covariance = covariance;
    this->most_recent_update_time = 0;
    this->previous_update_time = 0;
}

V Estimator::get_state() {
    return state;
}

M Estimator::get_covariance() {
    return covariance;
}

StatePackage Estimator::get_internals() {
    return {state, covariance, most_recent_update_time};
}

void Estimator::updateInternals(StatePackage package) {
    state = package.state;
    covariance = package.covariance;
    previous_update_time = most_recent_update_time;
    most_recent_update_time = package.update_time;
}

void Estimator::updateInternals(SimpleStatePackage package) {
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

M Estimator::state_mask_to_matrix(std::vector<std::string> state_mask) {
    M mask_matrix = M::Zero(S, S);
    for (std::string& state: state_mask) {
        mask_matrix(ckf::state::string_state.at(state), ckf::state::string_state.at(state)) = 1;
    }
    return mask_matrix;
}
