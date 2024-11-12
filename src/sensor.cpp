#include "sensor.h"

Sensor::Sensor(V state, M covariance) {
  this->state = state;
  this->covariance = covariance;
}

V Sensor::get_state() {
  return state;
}

M Sensor::get_covariance() {
  return covariance;
}

void Sensor::set_state(V state) {
  this->state = state;
}

void Sensor::set_covariance(M covariance) {
  this->covariance = covariance;
}

M Sensor::state_matrix_multiplier() {
  return MatrixXd::Identity(S, S);
}