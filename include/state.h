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

#define x__          0
#define y__          1
#define z__          2
#define roll__       3
#define pitch__      4
#define yaw__        5
#define d_x__        6
#define d_y__        7
#define d_z__        8
#define d_roll__     9
#define d_pitch__    10
#define d_yaw__      11
#define d2_x__       12
#define d2_y__       13
#define d2_z__       14
#define tau__        15
#define d_tau__      16
#define d2_tau__     17

#define x_          state[ x__      ]
#define y_          state[ y__      ]
#define z_          state[ z__      ]
#define roll_       state[ roll__   ]
#define pitch_      state[ pitch__  ]
#define yaw_        state[ yaw__    ]
#define d_x_        state[ d_x__    ]
#define d_y_        state[ d_y__    ]
#define d_z_        state[ d_z__    ]
#define d_roll_     state[ d_roll__ ]
#define d_pitch_    state[ d_pitch__]
#define d_yaw_      state[ d_yaw__  ]
#define d2_x_       state[ d2_x__   ]
#define d2_y_       state[ d2_y__   ]
#define d2_z_       state[ d2_z__   ]
#define tau_        state[ tau__    ]
#define d_tau_      state[ d_tau__  ]
#define d2_tau_     state[ d2_tau__ ]

#define _x_         this->x_          
#define _y_         this->y_          
#define _z_         this->z_          
#define _roll_      this->roll_       
#define _pitch_     this->pitch_      
#define _yaw_       this->yaw_        
#define _d_x_       this->d_x_        
#define _d_y_       this->d_y_        
#define _d_z _      this->d_z_        
#define _d_roll_    this->d_roll_     
#define _d_pitch_   this->d_pitch_    
#define _d_yaw_     this->d_yaw_      
#define _d2_x_      this->d2_x_       
#define _d2_y_      this->d2_y_       
#define _d2_z_      this->d2_z_       
#define _tau_       this->tau_        
#define _d_tau_     this->d_tau_      
#define _d2_tau_    this->d2_tau_     

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