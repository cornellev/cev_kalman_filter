#pragma once

#include <utility>
#include "state.h"

class UpdateModel : public HasState {
  private:
    M base_process_covariance;

  public:
    /**
     * Base class for an EKF state update model
     *
     * @param state Start state
     * @param covariance Start covariance
     * @param base_process_covariance Process covariance over time
     */
    UpdateModel(V state, M covariance, M process_covariance);

    /**
     * Next state and covariance without adjusting with a sensor update.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Next state and covariance
     */
    std::pair<V, M> predict(double dt);

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
      V sensor_state,
      M sensor_variance,
      M sensor_transformation,
      double dt
    );

    /**
     * Perform a model update step on `state` with time `dt`.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Updated system state
    */
    virtual V update_step(V state, double dt) = 0;

    /**
     * Jacobian matrix of a model update step on `state` with time `dt`.
     *
     * @param state Last system state
     * @param dt Time elapsed since last state update
     *
     * @return Jacobian matrix of state update step
     */
    virtual M update_jacobian(V state, double dt) = 0;

    /**
     * Transformed version of the model Jacobian matrix for the state of a given sensor.
     * @param state Last system state
     * @param dt Time elapsed since last state update
     * @param sensor_transformation Transformation from model state to sensor state
     *
     * @return Sensor-specific model Jacobian matrix of state update step
     */
    M sensor_jacobian(V state, double dt, M sensor_transformation);

    /**
     * Gives process covariance over elapsed time dt.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Process covariance matrix
     */
    M process_covariance(double dt);

    /**
     * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of this model.
     * 
     * @return Multiplier matrix
     */
    virtual M state_matrix_multiplier();

    // Aliases
    inline V x() { return state; };
    inline M P() { return covariance; };
    inline V f(V x, double dt) { return update_step(x, dt); };
    inline M F(V x, double dt) { return update_jacobian(x, dt); };
    inline M H(V x, double dt, M tr) { return sensor_jacobian(x, dt, tr); };
    inline M Q(double dt) { return process_covariance(dt); };
};