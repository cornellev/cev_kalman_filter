#include "sensor.h"

Sensor::Sensor(
    V state, 
    M covariance,
    std::vector<Listener> dependents
) : Updater(state, covariance, dependents) {};