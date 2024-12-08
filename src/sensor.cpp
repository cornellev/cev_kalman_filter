#include "sensor.h"

using namespace ckf;

Sensor::Sensor(V state, M covariance, std::vector<std::shared_ptr<Model>> dependents)
    : Estimator(state, covariance) {
    models = dependents;
}

void Sensor::bind_to(std::shared_ptr<Model> model) {
    models.push_back(model);
}

void Sensor::update_dependents() {
    Estimator* self = this;

    for (std::shared_ptr<Model> model: models) {
        model->estimate_update(*self);
    }
}