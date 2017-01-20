//
// Created by manuelli on 1/20/17.
//

#include "residual_detector.h"
#include "drake/common/drake_path.h"
#include "yaml-cpp/yaml.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "residual_detector_lcm_wrapper.h"

namespace residual_detector{
  ResidualDetectorConfig parseResidualDetectorConfig(std::string path_to_file_from_drake_root){
    std::string full_path_to_file = drake::GetDrakePath() + path_to_file_from_drake_root;
    std::cout << "full path to file: " << full_path_to_file << std::endl;
    YAML::Node config_yaml = YAML::LoadFile(full_path_to_file);
    YAML::Node robot_data_yaml = config_yaml["robot_data"];
    ResidualDetectorConfig residual_detector_config;


    residual_detector_config.urdf_filename = drake::GetDrakePath() + robot_data_yaml["urdf"].as<std::string>();
    residual_detector_config.robot_type = robot_data_yaml["robot_type"].as<std::string>();
    residual_detector_config.residual_gain = config_yaml["residual_detector"]["gain"].as<double>();

    return residual_detector_config;
  }

  ResidualDetector::ResidualDetector(ResidualDetectorConfig &residual_detector_config) {
    residual_detector_config_ = residual_detector_config;

    // create the RBT and add the urdf to it
    rigid_body_tree_ = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        residual_detector_config_.urdf_filename,
        drake::multibody::joints::kFixed,
        nullptr /* weld to frame */, rigid_body_tree_.get());
  }
}

