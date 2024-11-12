#pragma once

#include "state.h"

class Sensor : public HasState {
    public:
        /**
         * Base class for a sensor model
         *
         * @param state Current state
         * @param covariance Covariance
         */
        Sensor(V state, M covariance);
    
        /**
         * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of this model.
         * 
         * @return Multiplier matrix
         */
        virtual M state_matrix_multiplier();
};