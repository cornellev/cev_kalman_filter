#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "update_model.h"
#include "sensor.h"

using std::placeholders::_1;

class AckermannModel : public UpdateModel {
  // State: [x, y, x', y', yaw, steering_angle]

  private:
    double wheelbase;
    M multiplier;

  public:
    AckermannModel(
      V state,
      M covariance,
      M process_covariance,
      double wheelbase
    ) : UpdateModel(state, covariance, process_covariance) {
      this->wheelbase = wheelbase;

      multiplier(x__, x__) = 1;
      multiplier(y__, y__) = 1;
      multiplier(d_x__, d_x__) = 1;
      multiplier(d_y__, d_y__) = 1;
      multiplier(yaw__, yaw__) = 1;
      multiplier(tau__, tau__) = 1;
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

    M update_jacobian(V state, double dt) {
      M F_k = MatrixXd::Identity(S, S);

      double p_yaw_dx = dt * sin(tau_) / wheelbase;
      double p_yaw_tau = dt * d_x_ * cos(tau_) / wheelbase;

      double p_x_dx = dt * cos(yaw_ + tau_);
      double p_x_yaw = -dt * d_x_ * sin(yaw_ + tau_);
      double p_x_tau = -dt * d_x_ * sin(yaw_ + tau_);

      double p_y_dx = dt * sin(yaw_ + tau_);
      double p_y_yaw = dt * d_x_ * cos(yaw_ + tau_);
      double p_y_tau = dt * d_x_ * cos(yaw_ + tau_);

      F_k(x__, d_x__) = p_x_dx;
      F_k(x__, yaw__) = p_x_yaw;
      F_k(x__, tau__) = p_x_tau;

      F_k(y__, d_x__) = p_y_dx;
      F_k(y__, yaw__) = p_y_yaw;
      F_k(y__, tau__) = p_y_tau;

      F_k(yaw__, d_x__) = p_yaw_dx;
      F_k(yaw__, tau__) = p_yaw_tau;

      return F_k;
    }

    M state_matrix_multiplier() {
      return multiplier;
    }
};

class IMUSensor : public Sensor {
  private:
    M multiplier;

  public:
    IMUSensor(V state, M covariance) : Sensor(state, covariance) {
      multiplier(d_x__, d_x__) = 1;
      multiplier(d_y__, d_y__) = 1;
      multiplier(yaw__, yaw__) = 1;
    }

    M state_matrix_multiplier() {
      return multiplier;
    }
};

class OdomSensor : public Sensor {
  private:
    M multiplier;

  public:
    OdomSensor(V state, M covariance) : Sensor(state, covariance) {
      multiplier(d_x__, d_x__) = 1;
      multiplier(tau__, tau__) = 1;
    }

    M state_matrix_multiplier() {
      return multiplier;
    }
};

class AckermannEkfNode : public rclcpp::Node {
  public:
    AckermannEkfNode() : Node("AckermannEkfNode") {}

  private:
    AckermannModel model = AckermannModel(
      V::Zero(),
      M::Identity() * .1,
      M::Identity() * .1,
      1.0
    );

    IMUSensor imu = IMUSensor(
      V::Zero(),
      M::Identity() * .1
    );

    OdomSensor odom = OdomSensor(
      V::Zero(),
      M::Identity() * .1
    );
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannEkfNode>());
  rclcpp::shutdown();
  return 0;
}