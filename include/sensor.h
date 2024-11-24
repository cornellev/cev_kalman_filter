#pragma once

#include "model.h"

class Sensor : public Estimator {
    protected:
        std::vector<std::shared_ptr<Model>> models;

    public:
        /**
         * Base class for a sensor model
         *
         * @param state Current state
         * @param covariance Covariance
         * @param dependents Models that depend on this sensor
         */
        Sensor(
            V state, 
            M covariance,
            std::vector<std::shared_ptr<Model>> dependents = {}
        );

        /**
         * Bind a model to this sensor
         * 
         * @param model Model to bind
         */
        void bind_to(std::shared_ptr<Model> model);

        /**
         * Update all bound models
         */
        void update_dependents();
    
        /**
         * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of this model.
         * 
         * @return Multiplier matrix
         */
        virtual M state_matrix_multiplier() = 0;
};