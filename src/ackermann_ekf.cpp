#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "update_model.h"

using std::placeholders::_1;
using namespace Eigen;

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