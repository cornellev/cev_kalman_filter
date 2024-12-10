#include "standard_models.h"

namespace ckf {

    namespace standard_models {
        // ACKERMANN MODEL

        // Constructor definition
        AckermannModel::AckermannModel(V state, M covariance, M process_covariance,
            double wheelbase, std::vector<std::string> state_mask)
            : Model(state, covariance, process_covariance) {
            this->wheelbase = wheelbase;
            multiplier = state_mask_to_matrix(state_mask);
        }

        // Update step definition
        V AckermannModel::update_step(double time) {
            // Extract state definitions
            double x_ = state[state::x];
            double y_ = state[state::y];
            double d_x_ = state[state::d_x];
            double d_y_ = state[state::d_y];
            double d2_x_ = state[state::d2_x];
            double d2_y_ = state[state::d2_y];
            double yaw_ = state[state::yaw];
            double tau_ = state[state::tau];
            double d_tau_ = state[state::d_tau];

            double dt = time - most_recent_update_time;

            double d_yaw = d_x_ * (sin(tau_) / wheelbase) * dt;
            double new_yaw = yaw_ + d_yaw;

            double new_d_x = d_x_ + d2_x_ * dt;
            double new_d_y = d_y_ + d2_y_ * dt;
            double new_tau = tau_ + d_tau_ * dt;

            double new_x = x_ + d_x_ * cos(yaw_ + tau_) * dt;
            double new_y = y_ + d_y_ * sin(yaw_ + tau_) * dt;

            V new_state = this->state;

            new_state[state::x] = new_x;
            new_state[state::y] = new_y;
            new_state[state::d_x] = new_d_x;
            new_state[state::d_y] = new_d_y;
            new_state[state::yaw] = new_yaw;
            new_state[state::tau] = new_tau;

            return new_state;
        }

        // Update Jacobian definition
        M AckermannModel::update_jacobian(double dt) {
            // Extract state definitions
            double x_ = state[state::x];
            double y_ = state[state::y];
            double d_x_ = state[state::d_x];
            double d_y_ = state[state::d_y];
            double d2_x_ = state[state::d2_x];
            double d2_y_ = state[state::d2_y];
            double yaw_ = state[state::yaw];
            double tau_ = state[state::tau];

            M F_k = MatrixXd::Identity(S, S);

            F_k(state::x, state::d_x) = dt * cos(yaw_ + tau_);
            F_k(state::x, state::yaw) = -dt * d_x_ * sin(yaw_ + tau_);
            F_k(state::x, state::tau) = -dt * d_x_ * sin(yaw_ + tau_);

            F_k(state::y, state::d_x) = dt * sin(yaw_ + tau_);
            F_k(state::y, state::yaw) = dt * d_x_ * cos(yaw_ + tau_);
            F_k(state::y, state::tau) = dt * d_x_ * cos(yaw_ + tau_);

            F_k(state::yaw, state::d_x) = dt * sin(tau_) / wheelbase;
            F_k(state::yaw, state::tau) = dt * d_x_ * cos(tau_) / wheelbase;

            F_k(state::d_x, state::d2_x) = dt;
            F_k(state::d_y, state::d2_y) = dt;

            return F_k;
        }

        // Get state matrix multiplier definition
        M AckermannModel::state_matrix_multiplier() {
            return multiplier;
        }

        // CARTESIAN MODEL

        CartesianModel::CartesianModel(V state, M covariance, M process_covariance,
            double wheelbase, std::vector<std::string> state_mask)
            : Model(state, covariance, process_covariance) {
            multiplier = state_mask_to_matrix(state_mask);
        }

        // Update step definition
        V CartesianModel::update_step(double time) {
            // Extract state definitions
            double x_ = state[state::x];
            double y_ = state[state::y];
            double d_x_ = state[state::d_x];
            double d_y_ = state[state::d_y];
            double d2_x_ = state[state::d2_x];
            double d2_y_ = state[state::d2_y];
            double yaw_ = state[state::yaw];
            double d_yaw_ = state[state::d_yaw];

            double dt = time - most_recent_update_time;

            double new_yaw = yaw_ + d_yaw_ * dt;

            double new_d_x = d_x_ + d2_x_ * dt;
            double new_d_y = d_y_ + d2_y_ * dt;

            double new_x = x_ + d_x_ * cos(yaw_) * dt + .5 * d2_x_ * cos(yaw_) * dt * dt;
            double new_y = y_ + d_y_ * sin(yaw_) * dt + .5 * d2_y_ * sin(yaw_) * dt * dt;

            V new_state = this->state;

            new_state[state::x] = new_x;
            new_state[state::y] = new_y;
            new_state[state::d_x] = new_d_x;
            new_state[state::d_y] = new_d_y;
            new_state[state::yaw] = new_yaw;

            return new_state;
        }

        // Update Jacobian definition
        M CartesianModel::update_jacobian(double dt) {
            // Extract state definitions
            double x_ = state[state::x];
            double y_ = state[state::y];
            double d_x_ = state[state::d_x];
            double d_y_ = state[state::d_y];
            double d2_x_ = state[state::d2_x];
            double d2_y_ = state[state::d2_y];
            double yaw_ = state[state::yaw];
            double tau_ = state[state::tau];

            M F_k = MatrixXd::Identity(S, S);

            F_k(state::x, state::d_x) = dt;
            F_k(state::y, state::d_y) = dt;

            F_k(state::x, state::d2_x) = .5 * dt * dt;
            F_k(state::y, state::d2_y) = .5 * dt * dt;

            F_k(state::x, state::yaw) = -dt * d_x_ * sin(yaw_);
            F_k(state::y, state::yaw) = dt * d_y_ * cos(yaw_);

            F_k(state::d_x, state::d2_x) = dt;
            F_k(state::d_y, state::d2_y) = dt;

            F_k(state::yaw, state::d_yaw) = dt;

            return F_k;
        }

        // Get state matrix multiplier definition
        M CartesianModel::state_matrix_multiplier() {
            return multiplier;
        }
    }
}
