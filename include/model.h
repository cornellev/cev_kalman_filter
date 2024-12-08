#pragma once

#include <utility>
#include <memory>
#include "estimator.h"

namespace ckf {
    class Model : public Estimator {
    protected:
        M base_process_covariance;

        std::vector<std::shared_ptr<Model>> models;

        /**
         * Perform a model update step up to `time`.
         *
         * @param time Time to update to
         *
         * @return Updated system state
         */
        virtual V update_step(double time) = 0;

        /**
         * Jacobian matrix of a model update step on `state` with time `dt`.
         *
         * @param dt Time since last update
         *
         * @return Jacobian matrix of state update step
         */
        virtual M update_jacobian(double dt) = 0;

        /**
         * Transformed version of the model Jacobian matrix for the state of a given sensor.
         *
         * @param estimate New sensor estimate
         *
         * @return Sensor-specific model Jacobian matrix of state update step
         */
        M sensor_jacobian(Estimator& estimate);

        /**
         * Gives process covariance over elapsed time dt.
         *
         * @param time Time since last update
         *
         * @return Process covariance matrix
         */
        M process_covariance(double dt);

        /**
         * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of
         * this model.
         *
         * @return Multiplier matrix
         */
        virtual M state_matrix_multiplier() = 0;

    public:
        /**
         * Base class for an EKF state update model
         *
         * @param state Start state
         * @param covariance Start covariance
         * @param base_process_covariance Process covariance over time
         * @param dependents Models that depend on this model
         *
         */
        Model(V state, M covariance, M process_covariance,
            std::vector<std::shared_ptr<Model>> dependents = {});

        /**
         * Next state and covariance without adjusting with a sensor update.
         *
         * @param time Time to predict to
         *
         * @return Next state and covariance
         */
        std::pair<V, M> predict(double time);

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
        void estimate_update(Estimator& estimate);

        /**
         * Bind a model to this updater
         *
         * @param model Model to bind
         */
        void bind_to(std::shared_ptr<Model> model);

        void force_state(V state) {
            this->state = state;
        }

        /**
         * Update all bound models
         */
        void update_dependents();
    };
}