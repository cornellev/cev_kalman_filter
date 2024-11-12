#include "update_model.h"


UpdateModel::UpdateModel(
  V state,
  M covariance,
  M process_covariance
) : HasState(state, covariance) {
  this->base_process_covariance = process_covariance;
}

std::pair<V, M> UpdateModel::predict(double dt) {
  M F_k = update_jacobian(state, dt);
  M Q_k = process_covariance(dt);

  return std::make_pair(update_step(state, dt), F_k * covariance * F_k.transpose() + Q_k);
}

void UpdateModel::update(double dt) {
  std::pair<V, M> prediction = predict(dt);
  state = prediction.first;
  covariance = prediction.second;
}

void UpdateModel::sensor_update(
  V sensor_state,
  M sensor_variance,
  M sensor_transformation,
  double dt
) {
  std::pair<V, M> prediction = predict(dt);
  V state = prediction.first;
  M covariance = prediction.second;

  M H_k = sensor_jacobian(state, dt, sensor_transformation);
  M R_k = sensor_variance;

  V predicted_sensor = sensor_transformation * state;
  V real_sensor = sensor_state;

  V y_k = real_sensor - predicted_sensor;

  M S_k = H_k * covariance * H_k.transpose() + R_k;
  M K_k = covariance * H_k.transpose() * S_k.inverse();

  state = state + K_k * y_k;
  covariance = (
    MatrixXd::Identity(covariance.rows(), covariance.cols()
  ) - K_k * H_k) * covariance;
}

M UpdateModel::process_covariance(double dt) {
  return base_process_covariance * dt;
}

M UpdateModel::sensor_jacobian(
  V state,
  double dt,
  M sensor_transformation
) {
  return update_jacobian(state, dt) * sensor_transformation;
}

M UpdateModel::state_matrix_multiplier() {
  return MatrixXd::Identity(S, S);
}