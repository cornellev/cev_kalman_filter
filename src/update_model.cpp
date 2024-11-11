#include "update_model.h"

template <int S>
UpdateModel<S>::UpdateModel(
  Vector<double, S> state,
  Matrix<double, S, S> covariance,
  Matrix<double, S, S> process_covariance
) {
  this->state = state;
  this->covariance = covariance;
  this->base_process_covariance = process_covariance;
}

template <int S>
std::pair<Vector<double, S>, Matrix<double, S, S>> UpdateModel<S>::predict(double dt) {
  Matrix<double, S, S> F_k = update_jacobian(state, dt);
  Matrix<double, S, S> Q_k = process_covariance(dt);

  return std::make_pair(update_step(state, dt), F_k * covariance * F_k.transpose() + Q_k);
}

template <int S>
void UpdateModel<S>::update(double dt) {
  std::pair<Vector<double, S>, Matrix<double, S, S>> prediction = predict(dt);
  state = prediction.first;
  covariance = prediction.second;
}

template <int S>
void UpdateModel<S>::sensor_update(
  Vector<double, S> sensor_state,
  Matrix<double, S, S> sensor_variance,
  Matrix<double, S, S> sensor_transformation,
  double dt
) {
  std::pair<Vector<double, S>, Matrix<double, S, S>> prediction = predict(dt);
  Vector<double, S> state = prediction.first;
  Matrix<double, S, S> covariance = prediction.second;

  Matrix<double, S, S> H_k = sensor_jacobian(state, dt, sensor_transformation);
  Matrix<double, S, S> R_k = sensor_variance;

  Vector<double, S> predicted_sensor = sensor_transformation * state;
  Vector<double, S> real_sensor = sensor_state;

  Vector<double, S> y_k = real_sensor - predicted_sensor;

  Matrix<double, S, S> S_k = H_k * covariance * H_k.transpose() + R_k;
  Matrix<double, S, S> K_k = covariance * H_k.transpose() * S_k.inverse();

  state = state + K_k * y_k;
  covariance = (
    MatrixXd::Identity(covariance.rows(), covariance.cols()
  ) - K_k * H_k) * covariance;
}

template <int S>
Matrix<double, S, S> UpdateModel<S>::process_covariance(double dt) {
  return base_process_covariance * dt;
}

template <int S>
Matrix<double, S, S> UpdateModel<S>::sensor_jacobian(
  Vector<double, S> state,
  double dt,
  Matrix<double, S, S> sensor_transformation
) {
  return update_jacobian(state, dt) * sensor_transformation;
}

template <int S>
Matrix<double, S, S> UpdateModel<S>::get_state() {
  return state;
}

template <int S>
Matrix<double, S, S> UpdateModel<S>::get_covariance() {
  return covariance;
}