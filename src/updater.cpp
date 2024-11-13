#include "updater.h"

Updater::Updater(
    V state, 
    M covariance, 
    double last_update_time, 
    bool initialized,
    std::vector<Listener> dependents
) : Estimator(state, covariance, last_update_time, initialized) {
    models = dependents;
}

void Updater::bind_to(Listener model) {
    models.push_back(model);
}

void Updater::update_dependents() {
    Estimator* self = this;

    for (Listener model : models) {
        model->estimate_update(*self);
    }
}