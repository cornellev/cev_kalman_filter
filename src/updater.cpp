#include "updater.h"

Updater::Updater(
    V state, 
    M covariance,
    std::vector<Listener> dependents
) : Estimator(state, covariance) {
    models = dependents;
}

void Updater::bind_to(Listener model) {
    models.push_back(model);
}

void Updater::update_dependents() {
    Estimator* self = this;

    // std::cout << this->name << " : " << self->get_state() << std::endl;

    for (Listener model : models) {
        model->estimate_update(*self);

        // std::cout << model->name << " : " << model->get_state() << std::endl;
    }
}