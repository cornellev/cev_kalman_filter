#include "update_model.h"

UpdateModel::UpdateModel(
  VectorXd state,
  MatrixXd covariance,
  MatrixXd process_covariance
) {
  this->state = state;
  this->covariance = covariance;
  this->base_process_covariance = process_covariance;
}

std::pair<VectorXd, MatrixXd> UpdateModel::predict(float dt) {
  MatrixXd F_k = update_jacobian(state, dt);
  MatrixXd Q_k = process_covariance(dt);

  return std::make_pair(update_step(state, dt), F_k * covariance * F_k.transpose() + Q_k);
}

void UpdateModel::update(float dt) {
  std::pair<VectorXd, MatrixXd> prediction = predict(dt);
  state = prediction.first;
  covariance = prediction.second;
}

void UpdateModel::sensor_update(
  VectorXd sensor_state,
  MatrixXd sensor_variance,
  MatrixXd sensor_transformation,
  float dt
) {
  std::pair<VectorXd, MatrixXd> prediction = predict(dt);
  VectorXd state = prediction.first;
  MatrixXd covariance = prediction.second;

  MatrixXd H_k = sensor_jacobian(state, dt, sensor_transformation);
  MatrixXd R_k = sensor_variance;

  VectorXd predicted_sensor = sensor_transformation * state;
  VectorXd real_sensor = sensor_state;

  VectorXd y_k = real_sensor - predicted_sensor;

  MatrixXd S_k = H_k * covariance * H_k.transpose() + R_k;
  MatrixXd K_k = covariance * H_k.transpose() * S_k.inverse();

  state = state + K_k * y_k;
  covariance = (
    MatrixXd::Identity(covariance.rows(), covariance.cols()
  ) - K_k * H_k) * covariance;
}

MatrixXd UpdateModel::process_covariance(float dt) {
  return base_process_covariance * dt;
}

MatrixXd UpdateModel::sensor_jacobian(
  VectorXd state,
  float dt,
  MatrixXd sensor_transformation
) {
  return update_jacobian(state, dt) * sensor_transformation;
}

MatrixXd UpdateModel::get_state() {
  return state;
}

MatrixXd UpdateModel::get_covariance() {
  return covariance;
}