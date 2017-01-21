#include "residual_detector_utils.h"
#include <iostream>

namespace residual_detector{
  RobotStateDecoder::RobotStateDecoder(std::shared_ptr<RigidBodyTree<double>> robot){
    num_positions_ = robot->get_num_positions();
    state_coordinate_names_.resize(num_positions_);
    for (int i=0; i < num_positions_; i++){
      state_coordinate_names_[i] = robot->get_position_name(i);
      state_coordinate_map_[state_coordinate_names_[i]] = i; // populate the map
      std::cout << state_coordinate_names_[i] << std::endl;
    }

  }

  DrakeRobotStateWithTorque RobotStateDecoder::decodeRobotStateMsg(const bot_core::robot_state_t *msg) {
    DrakeRobotStateWithTorque robot_state;
    robot_state.q.resize(num_positions_);
    robot_state.qd.resize(num_positions_);
    robot_state.torque.resize(num_positions_);

    robot_state.t = ((double) msg->utime) / 1e6;
    std::map<std::string, int>::iterator it;
    int idx;
    for(int i=0; i < msg->num_joints; i++){
      it = state_coordinate_map_.find(msg->joint_name[i]);
      if (it!=state_coordinate_map_.end()){
        idx = it->second;
        robot_state.q(idx) = msg->joint_position[i];
        robot_state.qd(idx) = msg->joint_velocity[i];
        robot_state.torque(idx) = msg->joint_effort[i];
      }
    }
    return robot_state;
  }
}