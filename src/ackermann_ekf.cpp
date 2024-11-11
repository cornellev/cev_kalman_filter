#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "update_model.h"

using std::placeholders::_1;
using namespace Eigen;

class AckermannModel : public UpdateModel<6> {
  // State: [x, y, x', y', yaw, steering_angle]
  private:
    float wheelbase;

  public:
    AckermannModel() : UpdateModel(
      VectorXd::Zero(6),
      MatrixXd::Identity(6, 6) * .3,
      MatrixXd::Identity(6, 6) * .1
    ) {
      wheelbase = 1.0;
    }

    Vector<double, 6> update_step(Vector<double, 6> state, double dt) {
      float d_yaw = state[2] * sin(state[4]) / wheelbase * dt;
      float new_yaw = state[4] + d_yaw;

      float new_x = state[0] + state[2] * cos(state[4] + state[5]) * dt;
      float new_y = state[1] + state[2] * sin(state[4] + state[5]) * dt;

      return Vector<double, 6> {
        new_x,
        new_y,
        state[2],
        state[3],
        new_yaw,
        state[5]
      };
    }

    Matrix<double, 6, 6> update_jacobian(Vector<double, 6> state, double dt) {
      Matrix<double, 6, 6> F_k;

      double d_x = state[2];
      double yaw = state[4];
      double tau = state[5];

      double p_yaw_dx = dt * sin(tau) / wheelbase;
      double p_yaw_tau = dt * d_x * cos(tau) / wheelbase;
      
      double p_x_dx = dt * cos(yaw + tau);
      double p_x_yaw = -dt * d_x * sin(yaw + tau);
      double p_x_tau = -dt * d_x * sin(yaw + tau);

      double p_y_dx = dt * sin(yaw + tau);
      double p_y_yaw = dt * d_x * cos(yaw + tau);
      double p_y_tau = dt * d_x * cos(yaw + tau);

      //     x   y    x'        y'   yaw        tau
      F_k << 1., 0.,  p_x_dx,   0.,  p_x_yaw,   p_x_tau,
             0., 1.,  p_y_dx,   0.,  p_y_yaw,   p_y_tau,
             0., 0.,  1.,       0.,  0.,        0.,
             0., 0.,  0.,       1.,  0.,        0.,
             0., 0.,  p_yaw_dx, 0.,  1.,        p_yaw_tau,
             0., 0.,  0.,       0.,  0.,        1.;

      return F_k;
    }
};

class AckermannEkfNode : public rclcpp::Node {
  public:
    AckermannEkfNode() : Node("AckermannEkfNode") {}
  // private:

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannEkfNode>());
  rclcpp::shutdown();
  return 0;
}