#pragma once

#include "updater.h"

class Sensor : public Updater {
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
            std::vector<Listener> dependents = {}
        );
    
        /**
         * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of this model.
         * 
         * @return Multiplier matrix
         */
        virtual M state_matrix_multiplier() = 0;
};