#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "update_model.h"

using std::placeholders::_1;
using namespace Eigen;

class AckermannModel : public UpdateModel {
  // State: [x, y, x', y', yaw, steering_angle]

  private:
    double wheelbase;

  public:
    AckermannModel(
      V state,
      M covariance,
      M process_covariance,
      double wheelbase
    ) : UpdateModel(state, covariance, process_covariance) {
      this->wheelbase = wheelbase;
    }

    V update_step(V state, double dt) {
      float d_yaw = d_x_ * sin(tau_) / wheelbase * dt;
      float new_yaw = yaw_ + d_yaw;

      float new_x = x_ + d_x_ * cos(yaw_ + tau_) * dt;
      float new_y = y_ + d_x_ * sin(yaw_ + tau_) * dt;

      V new_state;

      new_state[0] = new_x;
      new_state[1] = new_y;
      new_state[5] = new_yaw;

      return new_state;
    }

    // M update_jacobian(V state, double dt) {
    //   M F_k;

    //   double x = state[0];
    //   double y = state[1];
    //   double d_x = state[6];
    //   double d_y = state[7];
    //   double yaw = state[5];
    //   double tau = state[15];

    //   double p_yaw_dx = dt * sin(tau) / wheelbase;
    //   double p_yaw_tau = dt * d_x * cos(tau) / wheelbase;
      
    //   double p_x_dx = dt * cos(yaw + tau);
    //   double p_x_yaw = -dt * d_x * sin(yaw + tau);
    //   double p_x_tau = -dt * d_x * sin(yaw + tau);

    //   double p_y_dx = dt * sin(yaw + tau);
    //   double p_y_yaw = dt * d_x * cos(yaw + tau);
    //   double p_y_tau = dt * d_x * cos(yaw + tau);

    //   //     x   y    x'        y'   yaw        tau
    //   F_k << 1., 0.,  p_x_dx,   0.,  p_x_yaw,   p_x_tau,
    //          0., 1.,  p_y_dx,   0.,  p_y_yaw,   p_y_tau,
    //          0., 0.,  1.,       0.,  0.,        0.,
    //          0., 0.,  0.,       1.,  0.,        0.,
    //          0., 0.,  p_yaw_dx, 0.,  1.,        p_yaw_tau,
    //          0., 0.,  0.,       0.,  0.,        1.;

    //   return F_k;
    // }
};

class AckermannEkfNode : public rclcpp::Node {
  public:
    AckermannEkfNode() : Node("AckermannEkfNode") {}


  // private:
    // AckermannModel model = AckermannModel(
    //   Vector<double, 6> {0., 0., 0., 0., 0., 0.},
    //   Matrix<double, 6, 6>::Identity() * .1,
    //   Matrix<double, 6, 6>::Identity() * .1,
    //   1.0
    // );
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannEkfNode>());
  rclcpp::shutdown();
  return 0;
}