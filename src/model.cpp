#include "model.h"


Model::Model(
  V state,
  M covariance,
  M process_covariance,
  double last_update_time,
  bool initialized,
  std::vector<Listener> dependents
) : Updater(state, covariance, last_update_time, initialized, dependents) {
  this->base_process_covariance = process_covariance;
}

std::pair<V, M> Model::predict(double time) {
  M F_k = update_jacobian(time);
  M Q_k = process_covariance(time);

  return std::make_pair(update_step(time), F_k * covariance * F_k.transpose() + Q_k);
}

void Model::update(double time) {
  std::pair<V, M> prediction = predict(time);
  state = prediction.first;
  covariance = prediction.second;

  last_update_time = time;

  update_dependents();
}

void Model::estimate_update(
  Estimator estimate
) {
  std::pair<V, M> prediction = predict(estimate.get_last_update_time());
  V state = prediction.first;
  M covariance = prediction.second;

  M H_k = sensor_jacobian(estimate);
  M R_k = estimate.get_covariance();

  V predicted_sensor = estimate.state_matrix_multiplier() * state;
  V real_sensor = estimate.get_state();

  V y_k = real_sensor - predicted_sensor;

  M S_k = H_k * covariance * H_k.transpose() + R_k;
  M K_k = covariance * H_k.transpose() * S_k.inverse();

  state = state + K_k * y_k;
  covariance = (
    MatrixXd::Identity(covariance.rows(), covariance.cols()
  ) - K_k * H_k) * covariance;

  last_update_time = estimate.get_last_update_time();

  update_dependents();
}

M Model::process_covariance(double time) {
  return base_process_covariance * (time - last_update_time);
}

M Model::sensor_jacobian(Estimator estimate) {
  return estimate.state_matrix_multiplier() * update_jacobian(estimate.get_last_update_time());
}

M Model::state_matrix_multiplier() {
  return MatrixXd::Identity(S, S);
}