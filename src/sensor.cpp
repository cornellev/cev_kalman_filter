#include "sensor.h"

Sensor::Sensor(V state, M covariance) : HasState(state, covariance) {}

M Sensor::state_matrix_multiplier() {
  return MatrixXd::Identity(S, S);
}