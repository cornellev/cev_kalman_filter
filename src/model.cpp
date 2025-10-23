#include "model.h"

using namespace ckf;

Model::Model(V state, M covariance, M process_covariance,
    std::vector<std::shared_ptr<Model>> dependents)
    : Estimator(state, covariance) {
    this->base_process_covariance = process_covariance;
    this->models = dependents;
}

std::pair<V, M> Model::predict(double time) {
    double dt = time - most_recent_update_time;
    M F_k = update_jacobian(dt);
    M Q_k = process_covariance(dt);

    return {update_step(time), F_k * covariance * F_k.transpose() + Q_k};
}

void Model::update(double time) {
    if (!ready_for_update(time)) {
        return;
    }

    // Update state with prediction with no sensor data.
    const auto prediction = predict(time);
    this->state = prediction.first;
    this->covariance = prediction.second;

    previous_update_time = most_recent_update_time;
    most_recent_update_time = time;

    update_dependents();
}

void Model::estimate_update(Estimator& estimate) {
    const double estimate_time = estimate.get_most_recent_update_time();

    if (!ready_for_update(estimate_time)) {
        return;
    }

    // Predict the model state at the given time
    std::pair<V, M> prediction = predict(estimate_time);
    V st = prediction.first;
    M cov = prediction.second;

    // Linearize and get sensor covariance
    M H_k = sensor_jacobian(estimate);
    M R_k = estimate.get_covariance();

    // innovation & innovation covariance
    V predicted_sensor = estimate.state_matrix_multiplier() * st;
    V real_sensor = estimate.get_state();
    V y_k = real_sensor - predicted_sensor;
    M S_k = H_k * cov * H_k.transpose() + R_k;

    // Kalman gain
    M K_k = cov * H_k.transpose() * S_k.inverse();

    // Apply the correction, update step
    this->state = st + K_k * y_k;
    this->covariance = (MatrixXd::Identity(cov.rows(), cov.cols()) - K_k * H_k) * cov;

    previous_update_time = most_recent_update_time;
    most_recent_update_time = estimate_time;

    // Update models depending on this model
    update_dependents();
}

M Model::process_covariance(double dt) {
    return base_process_covariance * dt;
}

M Model::sensor_jacobian(Estimator& estimate) {
    return estimate.state_matrix_multiplier() * update_jacobian(estimate.dt());
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

bool Model::ready_for_update(double new_time) {
    if (new_time < most_recent_update_time) {
        return false;
    }
    if (initialized) {
        return true;
    }

    previous_update_time = new_time;
    most_recent_update_time = new_time;
    initialized = true;
    return false;
}
