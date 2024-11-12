#pragma once

#include "state.h"

class Sensor : public HasState {
    private:
        V state;
        M covariance;
    
    public:
        /**
         * Base class for a sensor model
         *
         * @param state Current state
         * @param covariance Covariance
         */
        Sensor(V state, M covariance);
    
        /**
         * Current state of the sensor
         *
         * @return Current sensor state
         */
        V get_state();
    
        /**
         * Covariance of the sensor
         *
         * @return Sensor covariance
         */
        M get_covariance();
    
        /**
         * Set the state of the sensor
         *
         * @param state New sensor state
         */
        void set_state(V state);

        /**
         * Set the covariance of the sensor
         *
         * @param covariance New sensor covariance
         */
        void set_covariance(M covariance);
    
        /**
         * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of this model.
         * 
         * @return Multiplier matrix
         */
        virtual M state_matrix_multiplier();
};