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
      // multiplier(d_tau__, d_tau__) = 1;
    }

    V update_step(double time) {
      double dt = time - most_recent_update_time;

      double d_yaw = d_x_ * (sin(tau_) / wheelbase) * dt;
      double new_yaw = yaw_ + d_yaw;

      double new_d_x = d_x_ + d2_x_ * dt;
      double new_d_y = d_y_ + d2_y_ * dt;
      // double new_d_tau = d_tau_ + d2_tau_ * dt;
      double new_tau = tau_ + d_tau_ * dt;

      double new_x = x_ + d_x_ * cos(yaw_ + tau_) * dt;
      double new_y = y_ + d_x_ * sin(yaw_ + tau_) * dt;

      V new_state = this->state;

      new_state[x__] = new_x;
      new_state[y__] = new_y;
      new_state[d_x__] = new_d_x;
      new_state[d_y__] = new_d_y;
      new_state[yaw__] = new_yaw;
      new_state[tau__] = new_tau;
      // new_state[d_tau__] = new_d_tau;

      return new_state;
    }

    M update_jacobian(double dt) {
      M F_k = MatrixXd::Identity(S, S);

      F_k(x__, d_x__) = dt * cos(yaw_ + tau_);
      F_k(x__, yaw__) = -dt * d_x_ * sin(yaw_ + tau_);
      F_k(x__, tau__) = -dt * d_x_ * sin(yaw_ + tau_);

      F_k(y__, d_x__) = dt * sin(yaw_ + tau_);
      F_k(y__, yaw__) = dt * d_x_ * cos(yaw_ + tau_);
      F_k(y__, tau__) = dt * d_x_ * cos(yaw_ + tau_);

      F_k(yaw__, d_x__) = dt * sin(tau_) / wheelbase;
      F_k(yaw__, tau__) = dt * d_x_ * cos(tau_) / wheelbase;

      F_k(d_x__, d2_x__) = dt;
      F_k(d_y__, d2_y__) = dt;

      // F_k(tau__, d_tau__) = dt;
      // F_k(d_tau__, d2_tau__) = dt;

      return F_k;
    }

    M state_matrix_multiplier() {
      return multiplier;
    }
};

class IMUSensor : public RosSensor<sensor_msgs::msg::Imu> {
  private:
    double pos_mod(double angle) {
      return fmod(fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    }

  protected:
    bool initialized = false;
    bool relative = true;

    double initial_yaw;
    double last_reported_yaw = 0;
    double last_sensor_raw_yaw = 0;

  public:
    IMUSensor(
      V state,
      M covariance, 
      std::vector<std::shared_ptr<Model>> dependents,
      bool relative = true
    ) : RosSensor<sensor_msgs::msg::Imu>(
        state, 
        covariance,
        dependents
      ) 
    {
      multiplier(d2_x__, d2_x__) = 1.0;
      // multiplier(d2_y__, d2_y__) = 1.0;
      multiplier(yaw__, yaw__) = 1.0;

      this->relative = relative;
    }

    StatePackage msg_update(sensor_msgs::msg::Imu::SharedPtr msg) {
      this->name = "IMU";
      StatePackage estimate = get_internals();

      estimate.update_time = msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);

      estimate.state[d2_x__] = msg->linear_acceleration.x;
      // estimate.state[d2_y__] = msg->linear_acceleration.y;

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

      if (relative && !initialized) {
        last_sensor_raw_yaw = yaw;
        last_reported_yaw = 0.0;
        initialized = true;

        estimate.state[yaw__] = 0.0;
        return estimate;
      }

      double modded_yaw_diff = fmod(yaw - last_sensor_raw_yaw, 2 * M_PI);

      if (modded_yaw_diff > M_PI) {
        modded_yaw_diff -= 2 * M_PI;
      } else if (modded_yaw_diff < -M_PI) {
        modded_yaw_diff += 2 * M_PI;
      }

      last_reported_yaw += modded_yaw_diff;
      last_sensor_raw_yaw = yaw;

      estimate.state[yaw__] = last_reported_yaw;

      std::cout << "Yaw: " << yaw << " Last: " << last_reported_yaw << std::endl;

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

      estimate.update_time = msg->timestamp;

      estimate.state[d_x__] = msg->velocity;
      estimate.state[tau__] = msg->steering_angle;

      return estimate;
    }
};

class AckermannEkfNode : public rclcpp::Node {
  public:
    AckermannEkfNode() : Node("AckermannEkfNode") {
      V start_state = V::Zero();

      model->force_state(start_state);

      // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      //   "imu", 1, std::bind(&IMUSensor::msg_handler, &imu, _1)
      // );

      odom_sub = this->create_subscription<cev_msgs::msg::SensorCollect>(
        "sensor_collect", 1, std::bind(&OdomSensor::msg_handler, &odom, _1)
      );

      timer = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&AckermannEkfNode::timer_callback, this)
      );

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry/meow", 1);
    }

    void timer_callback() {
      double time = get_clock()->now().seconds();
      model->update(time);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = this->now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "meow_link";

      V state = model->get_state();

      odom_msg.pose.pose.position.x = state[x__];
      odom_msg.pose.pose.position.y = state[y__];

      odom_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion q = tf2::Quaternion();
      q.setRPY(0, 0, state[yaw__]);
      q = q.normalized();
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      odom_msg.twist.twist.linear.x = state[d_x__];
      odom_msg.twist.twist.linear.y = state[d_y__];
      odom_msg.twist.twist.angular.z = state[d_yaw__];

      odom_pub->publish(odom_msg);

      geometry_msgs::msg::TransformStamped transformStamped;

      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = "odom";
      transformStamped.child_frame_id = "meow_link";

      transformStamped.transform.translation.x = state[x__];
      transformStamped.transform.translation.y = state[y__];
      transformStamped.transform.translation.z = state[z__];

      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(transformStamped);
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
          M::Identity() * .05,
          M::Identity() * .05,
          .185
        )
    );

    IMUSensor imu = IMUSensor(
      V::Zero(),
      M::Identity() * .1,
      {model}
    );

    OdomSensor odom = OdomSensor(
      V::Zero(),
      M::Identity() * .05,
      {model}
    );
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannEkfNode>());
  rclcpp::shutdown();
  return 0;
}