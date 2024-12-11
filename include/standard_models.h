#include "model.h"

namespace ckf {
    namespace standard_models {
        // ACKERMANN MODEL

        class AckermannModel : public Model {
        private:
            double wheelbase;
            M multiplier = M::Zero();

        public:
            // Constructor
            AckermannModel(V state, M covariance, M process_covariance, double wheelbase,
                std::vector<std::string> state_mask = {"x", "y", "d_x", "d_y", "yaw", "tau"});

            V update_step(double time);
            M update_jacobian(double dt);
            M state_matrix_multiplier();
        };

        // CARTESIAN MODEL

        class CartesianModel : public Model {
        private:
            M multiplier = M::Zero();

        public:
            // Constructor
            CartesianModel(V state, M covariance, M process_covariance,
                std::vector<std::string> state_mask = {"x", "y", "d_x", "d_y", "yaw", "d_yaw"});

            V update_step(double time);
            M update_jacobian(double dt);
            M state_matrix_multiplier();
        };
    }
}