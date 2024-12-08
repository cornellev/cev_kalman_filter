#include "model.h"

using namespace ckf;

Model::Model(V state, M covariance, M process_covariance,
    std::vector<std::shared_ptr<Model>> dependents)
    : Estimator(state, covariance) {
    this->base_process_covariance = process_covariance;
    this->models = dependents;
}

std::pair<V, M> Model::predict(double time) {
    M F_k = update_jacobian(time - most_recent_update_time);
    M Q_k = process_covariance(time - most_recent_update_time);

    return std::make_pair(update_step(time), F_k * covariance * F_k.transpose() + Q_k);
}

void Model::update(double time) {
    if (time - most_recent_update_time < 0) {
        return;
    }

    if (!this->initialized) {
        this->previous_update_time = time;
        this->most_recent_update_time = time;
        this->initialized = true;
        return;
    }

    std::pair<V, M> prediction = predict(time);
    this->state = prediction.first;
    this->covariance = prediction.second;

    previous_update_time = most_recent_update_time;
    most_recent_update_time = time;

    update_dependents();
}

void Model::estimate_update(Estimator& estimate) {
    if (estimate.get_most_recent_update_time() - most_recent_update_time < 0.0) {
        return;
    }

    if (!this->initialized) {
        this->previous_update_time = estimate.get_most_recent_update_time();
        this->most_recent_update_time = estimate.get_most_recent_update_time();
        this->initialized = true;
        return;
    }

    std::pair<V, M> prediction = predict(estimate.get_most_recent_update_time());
    V st = prediction.first;
    M cov = prediction.second;

    M H_k = sensor_jacobian(estimate);
    M R_k = estimate.get_covariance();

    V predicted_sensor = estimate.state_matrix_multiplier() * st;
    V real_sensor = estimate.get_state();
    V y_k = real_sensor - predicted_sensor;

    M S_k = H_k * cov * H_k.transpose() + R_k;
    M K_k = cov * H_k.transpose() * S_k.inverse();

    this->state = st + K_k * y_k;

    this->covariance = (MatrixXd::Identity(cov.rows(), cov.cols()) - K_k * H_k) * cov;

    previous_update_time = most_recent_update_time;
    most_recent_update_time = estimate.get_most_recent_update_time();

    update_dependents();
}

M Model::process_covariance(double dt) {
    return base_process_covariance * dt;
}

M Model::sensor_jacobian(Estimator& estimate) {
    return estimate.state_matrix_multiplier()
           * update_jacobian(estimate.get_most_recent_update_time()
                             - estimate.get_previous_update_time());
}

void Model::bind_to(std::shared_ptr<Model> model) {
    models.push_back(model);
}

void Model::update_dependents() {
    Estimator* self = this;

    for (std::shared_ptr<Model> model: models) {
        model->estimate_update(*self);
    }
}