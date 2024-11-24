#include "model.h"


Model::Model(
  V state,
  M covariance,
  M process_covariance,
  std::vector<Listener> dependents
) : Updater(state, covariance, dependents) {
  this->base_process_covariance = process_covariance;
}

std::pair<V, M> Model::predict(double time) {
  M F_k = update_jacobian(time - most_recent_update_time);
  M Q_k = process_covariance(time - most_recent_update_time);

  return std::make_pair(update_step(time), F_k * covariance * F_k.transpose() + Q_k);
}

void Model::update(double time) {
  if (!this->initialized) {
    this->previous_update_time = time;
    this->most_recent_update_time = time;
    this->initialized = true;
    return;
  }

  std::pair<V, M> prediction = predict(time);
  state = prediction.first;
  covariance = prediction.second;

  previous_update_time = most_recent_update_time;
  most_recent_update_time = time;

  update_dependents();
}

void Model::estimate_update(
  Estimator &estimate
) {
  if (!this->initialized) {
    this->previous_update_time = estimate.get_most_recent_update_time();
    this->most_recent_update_time = estimate.get_most_recent_update_time();
    this->initialized = true;
    return;
  }

  std::pair<V, M> prediction = predict(estimate.get_most_recent_update_time());
  V state = prediction.first;
  M covariance = prediction.second;

  M H_k = sensor_jacobian(estimate);

  M R_k = estimate.get_covariance();

  V predicted_sensor = estimate.state_matrix_multiplier() * state;

  V real_sensor = estimate.get_state();

  V y_k = real_sensor - predicted_sensor;

  M S_k = H_k * covariance * H_k.transpose() + R_k;

  M K_k = covariance * H_k.transpose() * S_k.inverse();

  this->state = this->state + K_k * y_k;

  // std::cout << this->state << std::endl;

  this->covariance = (
    MatrixXd::Identity(this->covariance.rows(), this->covariance.cols()
  ) - K_k * H_k) * this->covariance;

  previous_update_time = most_recent_update_time;
  most_recent_update_time = estimate.get_most_recent_update_time();

  update_dependents();
}

M Model::process_covariance(double dt) {
  return base_process_covariance * dt;
}

M Model::sensor_jacobian(Estimator &estimate) {
  return estimate.state_matrix_multiplier() * update_jacobian(estimate.get_most_recent_update_time() - estimate.get_previous_update_time());
}