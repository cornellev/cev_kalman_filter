#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>

using namespace Eigen;

/*
State: [
  x,      y,      z,
  roll,   pitch,  yaw,
  x'      y',     z',
  roll',  pitch', yaw',
  x'',    y'',    z'',
  tau,    tau',   tau''
]

where tau = steering_angle
*/

namespace ckf {

    constexpr int S = 18;  // State size

    namespace state {
        const int x = 0;
        const int y = 1;
        const int z = 2;
        const int roll = 3;
        const int pitch = 4;
        const int yaw = 5;
        const int d_x = 6;
        const int d_y = 7;
        const int d_z = 8;
        const int d_roll = 9;
        const int d_pitch = 10;
        const int d_yaw = 11;
        const int d2_x = 12;
        const int d2_y = 13;
        const int d2_z = 14;
        const int tau = 15;
        const int d_tau = 16;
        const int d2_tau = 17;
    }

    // V represents a Vector of the state size
    using V = Vector<double, S>;
    // M represents a matrix of state x state size
    using M = Matrix<double, S, S>;

    /**
     * @struct StatePackage
     * Package for a estimate update
     */
    struct StatePackage {
        V state;
        M covariance;
        double update_time;
    };

    /**
     * @struct SimpleStatePackage
     * Struct for a simple state update, no covariance
     */
    struct SimpleStatePackage {
        V state;
        double update_time;
    };

    class Estimator {
    protected:
        V state;
        M covariance;

        double previous_update_time = 0.0;
        double most_recent_update_time = 0.0;

        bool initialized = false;

    public:
        /**
         * Base class for a model with a state and covariance
         *
         * @param state Start state
         * @param covariance Start covariance
         */
        Estimator(V state, M covariance);

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
         * Get the estimate internals
         *
         * @return Estimate internals
         */
        StatePackage get_internals();

        /**
         * Update the model estimate data with a simple state and time update
         *
         * @param package Package of state and time
         */
        void updateInternals(SimpleStatePackage package);

        /**
         * Update the model estimate data
         *
         * @param package Package of state, covariance, and time
         */
        void updateInternals(StatePackage package);

        /**
         * Get the time of the most recent update
         *
         * @return Time of most recent update
         */
        double get_most_recent_update_time();

        /**
         * Get the previous update time
         *
         * @return Previous update time
         */
        double get_previous_update_time();

        /**
         * Get the time since the most recent update
         *
         * @return Time since most recent update
         */
        double dt();

        /**
         * Matrix that, when left multiplied by a state vector,
         * gives a state matrix in the form of this model.
         *
         * @return Multiplier matrix
         */
        virtual M state_matrix_multiplier() = 0;

        static M state_mask_to_matrix(std::vector<bool> mask);
    };

}