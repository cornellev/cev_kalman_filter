#include "state.h"

HasState::HasState(V state, M covariance) {
  this->state = state;
  this->covariance = covariance;
}

V HasState::get_state() {
  return state;
}

M HasState::get_covariance() {
  return covariance;
}

void HasState::set_state(V state) {
  this->state = state;
}

void HasState::set_covariance(M covariance) {
  this->covariance = covariance;
}

M HasState::state_matrix_multiplier() {
  return MatrixXd::Identity(S, S);
}