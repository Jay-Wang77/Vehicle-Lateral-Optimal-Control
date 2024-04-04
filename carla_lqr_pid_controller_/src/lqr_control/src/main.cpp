#include <iostream>

#include "lqr_controller_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lqr_control");
  LQRControllerNode control_node;
  if (!control_node.init()) {
    std::cout << "fail to init lqr_control_node" << std::endl;
    return -1;
  }
  ros::spin();
  return 0;
}
