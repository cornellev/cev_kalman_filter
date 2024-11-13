#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "model.h"
#include "ros_sensor.h"

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cev_msgs/msg/sensor_collect.hpp"

using std::placeholders::_1;

class AckermannModel : public Model {
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
    ) : Model(state, covariance, process_covariance) {
      this->wheelbase = wheelbase;

      multiplier(x__, x__) = 1;
      multiplier(y__, y__) = 1;
      multiplier(d_x__, d_x__) = 1;
      multiplier(d_y__, d_y__) = 1;
      multiplier(yaw__, yaw__) = 1;
      multiplier(tau__, tau__) = 1;
    }

    V update_step(double time) {
      double dt = time - last_update_time;

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

    M update_jacobian(double time) {
      double dt = time - last_update_time;

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

class IMUSensor : public RosSensor<sensor_msgs::msg::Imu> {  
  public:
    IMUSensor(
      V state,
      M covariance, 
      std::vector<Listener> dependents
    ) 
    : RosSensor<sensor_msgs::msg::Imu>(
        state, 
        covariance,
        dependents
      ) 
    {
      multiplier(d2_x__, d2_x__) = 1;
      multiplier(d2_y__, d2_y__) = 1;
      multiplier(yaw__, yaw__) = 1;
    }

    void msg_update(sensor_msgs::msg::Imu msg) {
      // Convert IMU data to d2x, d2y, yaw
      d2_x_ = msg.linear_acceleration.x;
      d2_y_ = msg.linear_acceleration.y;

      // Get yaw from quaternion
      tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
      );
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      yaw_ = yaw;
    }
};

class OdomSensor : public RosSensor<cev_msgs::msg::SensorCollect> {
  private:
    M multiplier;

  public:
    OdomSensor(
      V state, 
      M covariance, 
      std::vector<Listener> dependents
    ) : RosSensor<cev_msgs::msg::SensorCollect>(
      state, 
      covariance,
      dependents
    ) {
      multiplier(d_x__, d_x__) = 1;
      multiplier(tau__, tau__) = 1;
    }

    M state_matrix_multiplier() {
      return multiplier;
    }

    void msg_update(cev_msgs::msg::SensorCollect msg) {
      d_x_ = msg.velocity;
      tau_ = msg.steering_angle;
    }
};

class AckermannEkfNode : public rclcpp::Node {
  public:
    AckermannEkfNode() : Node("AckermannEkfNode") {
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub = 
        this->create_subscription<sensor_msgs::msg::Imu>(
          "imu", 1, std::bind(&IMUSensor::msg_handler, &imu, _1)
        );

      rclcpp::Subscription<cev_msgs::msg::SensorCollect>::SharedPtr odom_sub = 
        this->create_subscription<cev_msgs::msg::SensorCollect>(
          "sensor_odom", 1, std::bind(&OdomSensor::msg_handler, &odom, _1)
        );

      timer = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&AckermannEkfNode::timer_callback, this)
      );

      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 1);
    }

    void timer_callback() {
      double time = get_clock()->now().seconds();

      model->update(time);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = this->now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";

      V state = model->get_state();

      odom_msg.pose.pose.position.x = state[x__];
      odom_msg.pose.pose.position.y = state[y__];
      odom_msg.pose.pose.position.z = 0;

      tf2::Quaternion q;
      q.setRPY(0, 0, state[yaw__]);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();

      odom_pub->publish(odom_msg);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<cev_msgs::msg::SensorCollect>::SharedPtr odom_sub;

    // Wall clock for publishing ackermann model state
    rclcpp::TimerBase::SharedPtr timer;

    // Odometry message publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    std::shared_ptr<AckermannModel> model = 
      std::make_shared<AckermannModel>(
        AckermannModel(
          V::Zero(),
          M::Identity() * .1,
          M::Identity() * .1,
          1.0
        )
      );

    IMUSensor imu = IMUSensor(
      V::Zero(),
      M::Identity() * .1,
      {model}
    );

    OdomSensor odom = OdomSensor(
      V::Zero(),
      M::Identity() * .1,
      {model}
    );
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannEkfNode>());
  rclcpp::shutdown();
  return 0;
}