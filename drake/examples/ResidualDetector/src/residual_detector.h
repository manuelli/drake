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


#ifndef DRAKE_DISTRO_RESIDUAL_DETECTOR_H
#define DRAKE_DISTRO_RESIDUAL_DETECTOR_H

namespace residual_detector{


  struct ResidualDetectorState{
    double t_prev;
    bool running;
    Eigen::VectorXd residual;
    Eigen::VectorXd gamma;
    Eigen::VectorXd integral;
    Eigen::VectorXd p_0; // momentum at t = 0;
  };

  struct ResidualDetectorInputArgs{
    DrakeRobotStateWithTorque robot_state;
  };

  struct ResidualDetectorConfig{
    std::string robot_type;
    std::string urdf_filename;
    double residual_gain;
  };

  ResidualDetectorConfig parseResidualDetectorConfig(std::string path_to_file_from_drake_root);

  class ResidualDetector {
  public:
    ResidualDetector(ResidualDetectorConfig & residual_detector_config);

  private:
    ResidualDetectorConfig residual_detector_config_;
    std::unique_ptr<RigidBodyTree<double>> rigid_body_tree_;
  };


}



#endif //DRAKE_DISTRO_RESIDUAL_DETECTOR_H
