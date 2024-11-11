#pragma once

#include <utility>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

template <int S>
class UpdateModel {
  private:
    Vector<double, S> state;
    Matrix<double, S, S> covariance;
    Matrix<double, S, S> base_process_covariance;

  public:
    /**
     * Base class for an EKF state update model
     *
     * @param state Start state
     * @param covariance Start state
     * @param base_process_covariance Process covariance over time
     */
    UpdateModel(Vector<double, S> state, Matrix<double, S, S> covariance, Matrix<double, S, S> process_covariance);

    /**
     * Next state and covariance without adjusting with a sensor update.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Next state and covariance
     */
    std::pair<Vector<double, S>, Matrix<double, S, S>> predict(double dt);

    /**
     * Update the state/covariance with no sensor estimate.
     * @param dt Time elapsed since last state update
     */
    void update(double dt);

    /**
     * Update the state/covariance with a sensor estimate.
     *
     * @param sensor_state Estimated state from sensor
     * @param sensor_variance Estimated variance of sensor
     * @param sensor_transformation Transformation from model state to sensor
     * @param dt Time elapsed since last state update
     */
    void sensor_update(
      Vector<double, S> sensor_state,
      Matrix<double, S, S> sensor_variance,
      Matrix<double, S, S> sensor_transformation,
      double dt
    );

    /**
     * Perform a model update step on `state` with time `dt`.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Updated system state
    */
    virtual Vector<double, S> update_step(Vector<double, S> state, double dt) = 0;

    /**
     * Jacobian matrix of a model update step on `state` with time `dt`.
     *
     * @param state Last system state
     * @param dt Time elapsed since last state update
     *
     * @return Jacobian matrix of state update step
     */
    virtual Matrix<double, S, S> update_jacobian(Vector<double, S> state, double dt) = 0;

    /**
     * Transformed version of the model Jacobian matrix for the state of a given sensor.
     * @param state Last system state
     * @param dt Time elapsed since last state update
     * @param sensor_transformation Transformation from model state to sensor state
     *
     * @return Sensor-specific model Jacobian matrix of state update step
     */
    Matrix<double, S, S> sensor_jacobian(Vector<double, S> state, double dt, Matrix<double, S, S> sensor_transformation);

    /**
     * Gives process covariance over elapsed time dt.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Process covariance matrix
     */
    Matrix<double, S, S> process_covariance(double dt);

    /**
     * Current state of the model
     *
     * @return Current model state
     */
    Matrix<double, S, S> get_state();

    /**
     * Current covariance of the model
     *
     * @return Current model covariance
     */
    Matrix<double, S, S> get_covariance();

    using V = Vector<double, S>;
    using M = Matrix<double, S, S>;
    using d = double;

    // Aliases
    inline V x() { return state; };
    inline M P() { return covariance; };
    inline V f(V x, d dt) { return update_step(x, dt); };
    inline M F(V x, d dt) { return update_jacobian(x, dt); };
    inline M H(V x, d dt, M tr) { return sensor_jacobian(x, dt, tr); };
    inline M Q(d dt) { return process_covariance(dt); };
};