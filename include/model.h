#pragma once

#include <utility>
#include "updater.h"

class Model : public Updateable, public Updater {
  protected:
    M base_process_covariance;

    /**
     * Next state and covariance without adjusting with a sensor update.
     *
     * @param time Time to predict to
     *
     * @return Next state and covariance
     */
    std::pair<V, M> predict(double time);

    /**
     * Perform a model update step on `state` with time `dt`.
     *
     * @param time Time to predict to
     *
     * @return Updated system state
    */
    virtual V update_step(double time) = 0;

    /**
     * Jacobian matrix of a model update step on `state` with time `dt`.
     *
     * @param time Time to predict to
     *
     * @return Jacobian matrix of state update step
     */
    virtual M update_jacobian(double time) = 0;

    /**
     * Transformed version of the model Jacobian matrix for the state of a given sensor.
     * 
     * @param estimate New sensor estimate
     *
     * @return Sensor-specific model Jacobian matrix of state update step
     */
    M sensor_jacobian(Estimator estimate);

    /**
     * Gives process covariance over elapsed time dt.
     *
     * @param time Time to predict to
     *
     * @return Process covariance matrix
     */
    M process_covariance(double time);

    /**
     * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of this model.
     * 
     * @return Multiplier matrix
     */
    virtual M state_matrix_multiplier();

  public:
    /**
     * Base class for an EKF state update model
     *
     * @param state Start state
     * @param covariance Start covariance
     * @param base_process_covariance Process covariance over time
     * @param last_update_time Time of last update
     * @param initialized Whether the model has been initialized
     * @param dependents Models that depend on this model
     * 
     */
    Model(
      V state,
      M covariance, 
      M process_covariance,
      double last_update_time = 0,
      bool initialized = false,
      std::vector<Listener> dependents = {}
    );

    /**
     * Update the state/covariance with no sensor estimate.
     * @param time Time to predict to
     */
    void update(double time);

    /**
     * Update the state/covariance with a sensor estimate.
     *
     * @param estimate New sensor estimate
     */
    void estimate_update(
      Estimator estimate
    );
};