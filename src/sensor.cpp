#include "sensor.h"

Sensor::Sensor(
    V state, 
    M covariance,
    double last_update_time,
    bool initialized,
    std::vector<Listener> dependents
) : Updater(state, covariance, last_update_time, initialized, dependents) {};