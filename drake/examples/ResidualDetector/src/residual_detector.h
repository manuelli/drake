//
// Created by manuelli on 1/20/17.
//

#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <string>
#include <memory>
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "residual_detector_utils.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/eigen_autodiff_types.h"


#ifndef DRAKE_DISTRO_RESIDUAL_DETECTOR_H
#define DRAKE_DISTRO_RESIDUAL_DETECTOR_H

namespace residual_detector{

  using namespace drake; // so we can easily call autodiff types
  typedef AutoDiffXd template_type;

  struct ResidualDetectorState{
    double t_prev;
    bool running;
    Eigen::VectorXd residual;
    Eigen::VectorXd gamma;
    Eigen::VectorXd integral;
    Eigen::VectorXd p_0; // momentum at t = 0;
    bool initialized;
  };

  struct ResidualDetectorInputArgs{
    std::shared_ptr<DrakeRobotStateWithTorque> robot_state;
  };

  struct ResidualDetectorConfig{
    std::string robot_type;
    std::string urdf_filename;
    double residual_gain;
    bool include_friction_torque;
    std::string publish_channel;
  };

  ResidualDetectorConfig parseResidualDetectorConfig(std::string path_to_file_from_drake_root);

  class ResidualDetector {
  public:
    ResidualDetector(ResidualDetectorConfig & residual_detector_config);
    void SetRobotState(std::shared_ptr<DrakeRobotStateWithTorque> robot_state);
    void ThreadLoop();
  private:
    ResidualDetectorConfig residual_detector_config_;
    std::unique_ptr<RigidBodyTree<double>> rigid_body_tree_;
    ResidualDetectorInputArgs input_args_;
    ResidualDetectorState internal_state_;
    std::mutex residual_args_lock_;
    std::unique_ptr<KinematicsCache<template_type >> cache_;
    bool new_state_available_;
    lcm::LCM lcm_;
    std::vector<std::string> state_coordinate_names_;

    void ResidualDetectorUpdate(ResidualDetectorState& residual_detector_state, const ResidualDetectorInputArgs args);

    void ResidualDetectorUpdateWrapper();

    void PublishResidualState(const ResidualDetectorState & residual_detector_state);
  };


}



#endif //DRAKE_DISTRO_RESIDUAL_DETECTOR_H
