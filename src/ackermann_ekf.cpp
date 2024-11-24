#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

#include "model.h"
#include "ros_sensor.h"

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cev_msgs/msg/sensor_collect.hpp"

#include <iostream>

using std::placeholders::_1;

class AckermannModel : public Model {
  // State: [x, y, x', y', yaw, steering_angle]

  private:
    double wheelbase;
    M multiplier = M::Zero();

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
      double dt = time - most_recent_update_time;

      // std::cout << "dx: " << _d_x_ << std::endl;

      float d_yaw = d_x_ * sin(tau_) / wheelbase * dt;
      float new_yaw = yaw_ + d_yaw;

      float new_x = x_ + d_x_ * cos(yaw_ + tau_) * dt;
      float new_y = y_ + d_x_ * sin(yaw_ + tau_) * dt;

      V new_state = this->state;

      new_state[0] = new_x;
      new_state[1] = new_y;
      new_state[5] = new_yaw;

      // std::cout << "new state: " << new_state.transpose() << std::endl;

      return new_state;
    }

    M update_jacobian(double dt) {
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
      std::vector<std::shared_ptr<Model>> dependents
    ) 
    : RosSensor<sensor_msgs::msg::Imu>(
        state, 
        covariance,
        dependents
      ) 
    {
      multiplier(d2_x__, d2_x__) = 1.0;
      multiplier(d2_y__, d2_y__) = 1.0;
      multiplier(yaw__, yaw__) = 1.0;
    }

    StatePackage msg_update(sensor_msgs::msg::Imu::SharedPtr msg) {
      this->name = "IMU";
      StatePackage estimate = get_internals();

      estimate.update_time = msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);

      estimate.state[d2_x__] = msg->linear_acceleration.x;
      estimate.state[d2_y__] = msg->linear_acceleration.y;

      // Get yaw from quaternion
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
      );
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      estimate.state[yaw__] = yaw;

      return estimate;
    }
};

class OdomSensor : public RosSensor<cev_msgs::msg::SensorCollect> {
  public:
    OdomSensor(
      V state, 
      M covariance, 
      std::vector<std::shared_ptr<Model>> dependents
    ) : RosSensor<cev_msgs::msg::SensorCollect>(
      state, 
      covariance,
      dependents
    ) {
      multiplier(d_x__, d_x__) = 1;
      multiplier(tau__, tau__) = 1;
    }

    StatePackage msg_update(cev_msgs::msg::SensorCollect::SharedPtr msg) {
      StatePackage estimate = get_internals();

      estimate.update_time = static_cast<double>(msg->stamp) / 1e9;

      estimate.state[d_x__] = msg->velocity;
      estimate.state[tau__] = msg->steering_angle;

      return estimate;
    }
};

class AckermannEkfNode : public rclcpp::Node {
  public:
    AckermannEkfNode() : Node("AckermannEkfNode") {
      V start_state = V::Zero();
      // start_state[x__] = 2.0;
      // start_state[d_x__] = 1.0;
      // start_state[tau__] = 30 * M_PI / 180.0;
      model->force_state(start_state);

      std::cout << "Start state: " << start_state.transpose() << std::endl;
      std::cout << "State: " << model->get_state().transpose() << std::endl;
      imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 1, std::bind(&IMUSensor::msg_handler, &imu, _1)
      );

      // odom_sub = this->create_subscription<cev_msgs::msg::SensorCollect>(
      //   "sensor_collect", 1, std::bind(&OdomSensor::msg_handler, &odom, _1)
      // );

      timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&AckermannEkfNode::timer_callback, this)
      );

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry/meow", 1);
    }

    void timer_callback() {
      // double time = get_clock()->now().seconds();

      // model->update(time);

      std::cout << "State: " << model->get_state().transpose() << std::endl;
      // std::cout << "Yaw in degrees: " << model->get_state()[yaw__] * 180.0 / M_PI << std::endl;
      // sensor_msgs::msg::Imu imu_msg;

      // model->update(time);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = this->now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";

      V state = model->get_state();

      odom_msg.pose.pose.position.x = state[x__];
      odom_msg.pose.pose.position.y = state[y__];

      odom_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion q = tf2::Quaternion();
      // odom_msg.pose.pose.orientation = tf2::createQuaternionMsgFromYaw(state[yaw__]);
      q.setY(state[yaw__]);
      q = q.normalized();
      // odom_msg.pose.pose.orientation.x = q.x();
      // odom_msg.pose.pose.orientation.y = q.y();
      // odom_msg.pose.pose.orientation.z = q.z();
      // odom_msg.pose.pose.orientation.w = q.w();

      odom_msg.twist.twist.linear.x = state[d_x__];
      odom_msg.twist.twist.linear.y = state[d_y__];
      odom_msg.twist.twist.angular.z = state[d_yaw__];

      // RCLCPP_INFO(this->get_logger(), "x: %f", model->get_covariance()(x__, x__));

      // odom_msg.pose.covariance[0] = model->get_covariance()(x__, x__);
      // odom_msg.pose.covariance[7] = model->get_covariance()(y__, y__);
      // odom_msg.pose.covariance[35] = model->get_covariance()(yaw__, yaw__);

      // odom_msg.twist.covariance[0] = model->get_covariance()(d_x__, d_x__);
      // odom_msg.twist.covariance[35] = model->get_covariance()(tau__, tau__);

      odom_pub->publish(odom_msg);

      geometry_msgs::msg::TransformStamped transformStamped;

      // Assuming you have the current state stored in the class
      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = "odom";
      transformStamped.child_frame_id = "base_link";

      // Replace these with the actual state variables
      transformStamped.transform.translation.x = state[x__];
      transformStamped.transform.translation.y = state[y__];
      transformStamped.transform.translation.z = 0.0; // Assuming 2D plane

      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      // tf_broadcaster_->sendTransform(transformStamped);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<cev_msgs::msg::SensorCollect>::SharedPtr odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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