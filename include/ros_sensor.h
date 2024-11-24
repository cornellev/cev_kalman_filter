#pragma once

#include "sensor.h"

template <typename T>
class RosSensor : public Sensor {
    protected:
        M multiplier = M::Zero();

        void new_time_handler(double time) {
            previous_update_time = most_recent_update_time;
            most_recent_update_time = time;
        }

    public:
        RosSensor(
            V state, 
            M covariance,
            std::vector<std::shared_ptr<Model>> dependents
        ) : Sensor(
                state, 
                covariance,
                dependents
            ) {}

        void msg_handler(typename T::SharedPtr msg) {
            StatePackage update = msg_update(msg);
            updateInternals(update);
            update_dependents();
        }

        /**
         * Update the state with a new message, and return the time of the new message
         */
        virtual StatePackage msg_update(typename T::SharedPtr msg) = 0;

        M state_matrix_multiplier() {
            return multiplier;
        }
};