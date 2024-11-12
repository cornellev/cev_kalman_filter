#include "update_model.h"

template <int S>
class UpdateModel {
public:
  UpdateModel(
    Vector<double, S> state,
    Matrix<double, S, S> covariance,
    Matrix<double, S, S> process_covariance
  ) {
    this->state = state;
    this->covariance = covariance;
    this->base_process_covariance = process_covariance;
  }

  std::pair<Vector<double, S>, Matrix<double, S, S>> predict(float dt) {
    Matrix<double, S, S> F_k = update_jacobian(state, dt);
    Matrix<double, S, S> Q_k = process_covariance(dt);

    return std::make_pair(update_step(state, dt), F_k * covariance * F_k.transpose() + Q_k);
  }

  void update(float dt) {
    std::pair<Vector<double, S>, Matrix<double, S, S>> prediction = predict(dt);
    state = prediction.first;
    covariance = prediction.second;
  }

  void sensor_update(
    Vector<double, S> sensor_state,
    Matrix<double, S, S> sensor_variance,
    Matrix<double, S, S> sensor_transformation,
    float dt
  ) {
    std::pair<Vector<double, S>, Matrix<double, S, S>> prediction = predict(dt);
    Vector<double, S> state = prediction.first;
    Matrix<double, S, S> covariance = prediction.second;

    Matrix<double, S, S> H_k = sensor_jacobian(state, dt, sensor_transformation);
    Matrix<double, S, S> R_k = sensor_variance;

    Vector<double, S> predicted_sensor = sensor_transformation * state;
    Vector<double, S> real_sensor = sensor_state;
  }

private:
  Vector<double, S> state;
  Matrix<double, S, S> covariance;
  Matrix<double, S, S> base_process_covariance;

  Matrix<double, S, S> update_jacobian(const Vector<double, S>& state, float dt);
  Matrix<double, S, S> process_covariance(float dt);
  Vector<double, S> update_step(const Vector<double, S>& state, float dt);
  Matrix<double, S, S> sensor_jacobian(const Vector<double, S>& state, float dt, const Matrix<double, S, S>& sensor_transformation);
};