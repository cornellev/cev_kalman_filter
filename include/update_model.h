#pragma once

using namespace Eigen;

class UpdateModel {
  public:
    /**
     * Perform a model update step on `state` with time `dt`.
     *
     * @param state Last system state
     * @param dt Time elapsed since last state update
     *
     * @return Updated system state
    */
    VectorXd update_step(VectorXd state, float dt);

    /**
     * Jacobian matrix of a model update step on `state` with time `dt`.
     *
     * @param state Last system state
     * @param dt Time elapsed since last state update
     *
     * @return Jacobian matrix of state update step
     */
    MatrixXd update_jacobian(VectorXd state, float dt);
};