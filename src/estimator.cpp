#include "estimator.h"

Estimator::Estimator(V state, M covariance, double last_update_time, bool initialized) {
  this->state = state;
  this->covariance = covariance;
  this->last_update_time = last_update_time;
  this->initialized = initialized;
}

V Estimator::get_state() {
  return state;
}

M Estimator::get_covariance() {
  return covariance;
}

void Estimator::set_state(V state, double time) {
  this->state = state;
  this->last_update_time = time;
  this->initialized = true;
}

void Estimator::set_covariance(M covariance) {
  this->covariance = covariance;
}

double Estimator::get_last_update_time() {
  return last_update_time;
}

bool Estimator::is_initialized() {
  return initialized;
}

M Estimator::state_matrix_multiplier() {
  return MatrixXd::Identity(S, S);
}