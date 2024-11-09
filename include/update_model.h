#pragma once

#include <utility>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class UpdateModel {
  private:
    VectorXd state;
    MatrixXd covariance;
    MatrixXd base_process_covariance;

  public:
    /**
     * Base class for an EKF state update model
     *
     * @param state Start state
     * @param covariance Start state
     * @param base_process_covariance Process covariance over time
     */
    UpdateModel(VectorXd state, MatrixXd covariance, MatrixXd process_covariance);

    /**
     * Next state and covariance without adjusting with a sensor update.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Next state and covariance
     */
    std::pair<VectorXd, MatrixXd> predict(float dt);

    /**
     * Update the state/covariance with no sensor estimate.
     * @param dt Time elapsed since last state update
     */
    void update(float dt);

    /**
     * Update the state/covariance with a sensor estimate.
     *
     * @param sensor_state Estimated state from sensor
     * @param sensor_variance Estimated variance of sensor
     * @param sensor_transformation Transformation from model state to sensor
     * @param dt Time elapsed since last state update
     */
    void sensor_update(
      VectorXd sensor_state,
      MatrixXd sensor_variance,
      MatrixXd sensor_transformation,
      float dt
    );

    /**
     * Perform a model update step on `state` with time `dt`.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Updated system state
    */
    virtual VectorXd update_step(VectorXd state, float dt) = 0;

    /**
     * Jacobian matrix of a model update step on `state` with time `dt`.
     *
     * @param state Last system state
     * @param dt Time elapsed since last state update
     *
     * @return Jacobian matrix of state update step
     */
    virtual MatrixXd update_jacobian(VectorXd state, float dt) = 0;

    /**
     * Transformed version of the model Jacobian matrix for the state of a given sensor.
     * @param state Last system state
     * @param dt Time elapsed since last state update
     * @param sensor_transformation Transformation from model state to sensor state
     *
     * @return Sensor-specific model Jacobian matrix of state update step
     */
    MatrixXd sensor_jacobian(VectorXd state, float dt, MatrixXd sensor_transformation);

    /**
     * Gives process covariance over elapsed time dt.
     *
     * @param dt Time elapsed since last state update
     *
     * @return Process covariance matrix
     */
    MatrixXd process_covariance(float dt);

    /**
     * Current state of the model
     *
     * @return Current model state
     */
    MatrixXd get_state();

    /**
     * Current covariance of the model
     *
     * @return Current model covariance
     */
    MatrixXd get_covariance();
};