#include "model.h"

namespace ckf {
    namespace standard_models {
        class AckermannModel : public Model {
        private:
            double wheelbase;
            M multiplier = M::Zero();

        public:
            // Constructor
            AckermannModel(V state, M covariance, M process_covariance, double wheelbase);

            V update_step(double time);
            M update_jacobian(double dt);
            M state_matrix_multiplier();
        };
    }
}