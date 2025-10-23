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
            double dt = time - most_recent_update_time;

            double speed    = state[state::d_x];
            double yaw      = state[state::yaw];
            double steering = state[state::tau];

            double heading = yaw + steering;
            double d_yaw   = speed * std::sin(steering) * dt / wheelbase;
            double new_tau = steering + state[state::d_tau] * dt;

            V new_state = this->state;

            new_state[ state::x ] += speed * std::cos(heading) * dt;
            new_state[ state::y ] += speed * std::sin(heading) * dt;
            new_state[state::d_x] =  speed + state[state::d2_x] * dt;
            new_state[state::d_y] =  state[state::d_y] + state[state::d2_y] * dt;
            new_state[state::yaw] += d_yaw;
            new_state[state::tau] =  new_tau;

            return new_state;
        }

        // Update Jacobian definition
        M AckermannModel::update_jacobian(double dt) {
            double speed    = state[state::d_x];
            double yaw      = state[state::yaw];
            double steering = state[state::tau];
            double heading  = yaw + steering;

            M F_k = MatrixXd::Identity(S, S);

            double cos_heading = std::cos(heading);
            double sin_heading = std::sin(heading);

            // x partials
            F_k(state::x, state::d_x) = dt * cos_heading;
            F_k(state::x, state::yaw) = -dt * speed * sin_heading;
            F_k(state::x, state::tau) = -dt * speed * sin_heading;

            // y partials
            F_k(state::y, state::d_x) = dt * sin_heading;
            F_k(state::y, state::yaw) = dt * speed * cos_heading;
            F_k(state::y, state::tau) = dt * speed * cos_heading;

            // yaw partials
            F_k(state::yaw, state::d_x) = dt * std::sin(steering) / wheelbase;
            F_k(state::yaw, state::tau) = dt * speed * std::cos(steering) / wheelbase;

            // velocity partials
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
            std::vector<std::string> state_mask)
            : Model(state, covariance, process_covariance) {
            multiplier = state_mask_to_matrix(state_mask);
        }

        // Update step definition
        V CartesianModel::update_step(double time) {
            double dt    = time - most_recent_update_time;
            double yaw   = state[state::yaw];
            double d_yaw = state[state::d_yaw];
            double speed = state[state::d_x];
            double accel = state[state::d2_x];

            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);

            V new_state = this->state;

            new_state[ state::x ] += speed * cos_yaw * dt + 0.5 * accel * cos_yaw * dt * dt;
            new_state[ state::y ] += speed * sin_yaw * dt + 0.5 * accel * sin_yaw * dt * dt;
            new_state[state::d_x]  = speed + accel * dt;
            // new_state[state::d_y] += state[state::d2_y] * dt;
            new_state[state::yaw] += d_yaw * dt;

            return new_state;
        }

        // Update Jacobian definition
        M CartesianModel::update_jacobian(double dt) {
            double yaw = state[state::yaw];
            double speed = state[state::d_x];

            M F_k = MatrixXd::Identity(S, S);

            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);

            // partials for x, y, d_x, yaw
            F_k(state::x, state::d_x) = cos_yaw * dt;
            F_k(state::y, state::d_y) = sin_yaw * dt;

            F_k(state::x, state::d2_x) = 0.5 * dt * dt;
            F_k(state::y, state::d2_x) = 0.5 * dt * dt;

            F_k(state::x, state::yaw) = -dt * speed * sin_yaw;
            F_k(state::y, state::yaw) = dt * speed * cos_yaw;

            F_k(state::d_x, state::d2_x) = dt;
            // F_k(state::d_y, state::d2_y) = dt;

            F_k(state::yaw, state::d_yaw) = dt;

            return F_k;
        }

        // Get state matrix multiplier definition
        M CartesianModel::state_matrix_multiplier() {
            return multiplier;
        }
    }
}
