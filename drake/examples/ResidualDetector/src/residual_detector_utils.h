//
// Created by manuelli on 1/20/17.
//

#ifndef DRAKE_DISTRO_RESIDUAL_DETECTOR_UTILS_H
#define DRAKE_DISTRO_RESIDUAL_DETECTOR_UTILS_H

#include <Eigen/Dense>
#include <memory>


#include <lcm/lcm-cpp.hpp>
#include "drake/multibody/rigid_body_tree.h"
#include "bot_core/robot_state_t.hpp"



namespace residual_detector{
  struct DrakeRobotStateWithTorque {
    // drake-ordered position and velocity vectors, with timestamp (in s)
    double t;
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd torque;
  };


  // for now this only supports fixed base robots
  class RobotStateDecoder{
  public:
    RobotStateDecoder(){}; // default constructor
    RobotStateDecoder(std::shared_ptr<RigidBodyTree<double>> robot);
    DrakeRobotStateWithTorque decodeRobotStateMsg(const bot_core::robot_state_t *msg);
  private:
    std::vector<std::string> state_coordinate_names_;
    std::map<std::string, int> state_coordinate_map_;
    int num_positions_;
//    int num_velocities_;

  };


  //utility class for threading

}

#endif //DRAKE_DISTRO_RESIDUAL_DETECTOR_UTILS_H
