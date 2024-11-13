#pragma once

#include "sensor.h"

template <typename T>
class RosSensor : public Sensor {
    protected:
        M multiplier = M::Identity();

    public:
        RosSensor(
            V state, 
            M covariance,
            std::vector<Listener> dependents
        ) : Sensor(
                state, 
                covariance,
                0,
                false,
                dependents
            ) {}

        void msg_handler(T msg) {
            msg_update(msg);
            update_dependents();
        }

        virtual void msg_update(T msg) = 0;

        M state_matrix_multiplier() {
            return multiplier;
        }
};