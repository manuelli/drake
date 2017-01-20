#include "residual_detector_utils.h"
#include <iostream>

namespace residual_detector{
  RobotStateDecoder::RobotStateDecoder(std::shared_ptr<RigidBodyTree<double>> robot){
    int num_positions = robot->get_num_positions();
    state_coordinate_names_.resize(num_positions);
    for (int i=0; i < num_positions; i++){
      state_coordinate_names_[i] = robot->get_position_name(i);
      std::cout << state_coordinate_names_[i] << std::endl;
    }

  }

  DrakeRobotStateWithTorque RobotStateDecoder::decodeRobotStateMsg(const bot_core::robot_state_t *msg) {
    DrakeRobotStateWithTorque robot_state;
    return robot_state;
  }
}