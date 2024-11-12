#pragma once

#include <eigen3/Eigen/Dense>

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

static constexpr int S = 18;

#define x_          state[0]
#define y_          state[1]
#define z_          state[2]
#define roll_       state[3]
#define pitch_      state[4]
#define yaw_        state[5]
#define d_x_        state[6]
#define d_y_        state[7]
#define d_z_        state[8]
#define d_roll_     state[9]
#define d_pitch_    state[10]
#define d_yaw_      state[11]
#define d2_x_       state[12]
#define d2_y_       state[13]
#define d2_z_       state[14]
#define tau_        state[15]
#define d_tau_      state[16]
#define d2_tau_     state[17]

#define _x_         this->state[0]
#define _y_         this->state[1]
#define _z_         this->state[2]
#define _roll_      this->state[3]
#define _pitch_     this->state[4]
#define _yaw_       this->state[5]
#define _d_x_       this->state[6]
#define _d_y_       this->state[7]
#define _d_z _      this->state[8]
#define _d_roll_    this->state[9]
#define _d_pitch_   this->state[10]
#define _d_yaw_     this->state[11]
#define _d2_x_      this->state[12]
#define _d2_y_      this->state[13]
#define _d2_z_      this->state[14]
#define _tau_       this->state[15]
#define _d_tau_     this->state[16]
#define _d2_tau_    this->state[17]

// V represents a Vector of the state size
using V = Vector<double, S>;
// M represents a matrix of state x state size
using M = Matrix<double, S, S>;

class HasState {
    public:
        /**
         * Current state of the sensor
         *
         * @return Current sensor state
         */
        virtual V get_state() = 0;
    
        /**
         * Covariance of the sensor
         *
         * @return Sensor covariance
         */
        virtual M get_covariance() = 0;
    
        /**
         * Set the state of the sensor
         *
         * @param state New sensor state
         */
        virtual void set_state(V state) = 0;

        /**
         * Set the covariance of the sensor
         *
         * @param covariance New sensor covariance
         */
        virtual void set_covariance(M covariance) = 0;
    
        /**
         * Matrix that, when left multiplied by a state vector, gives a state matrix in the form of this model.
         * 
         * @return Multiplier matrix
         */
        virtual M state_matrix_multiplier() = 0;
};