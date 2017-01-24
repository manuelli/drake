//
// Created by manuelli on 1/20/17.
//

#include "residual_detector.h"
#include "drake/common/drake_path.h"
#include "yaml-cpp/yaml.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "residual_detector_lcm_wrapper.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/autodiff.h"

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

  // constructor
  ResidualDetector::ResidualDetector(ResidualDetectorConfig &residual_detector_config) {
    residual_detector_config_ = residual_detector_config;

    // create the RBT and add the urdf to it
    rigid_body_tree_ = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        residual_detector_config_.urdf_filename,
        drake::multibody::joints::kFixed,
        nullptr /* weld to frame */, rigid_body_tree_.get());

    // initialize the ResidualDetectorState
    int num_positions = rigid_body_tree_->get_num_positions();
    this->internal_state_.residual.resize(num_positions);
    this->internal_state_.gamma.resize(num_positions);
    this->internal_state_.integral.resize(num_positions);
    this->internal_state_.p_0.resize(num_positions);
    this->internal_state_.initialized = false;


    // initialize boolean flags
    this->new_state_available_ = false;

    // initialize KinematicsCache
    cache_ = std::make_unique<KinematicsCache<template_type>>(this->rigid_body_tree_->CreateKinematicsCacheWithType<template_type>());
    std::cout << "cache->get_num_positions() " << cache_->get_num_positions() << std::endl;
  }

  // sets the robot_state field of ResidualArgs
  // grabs global lock to set the pointer
  void ResidualDetector::SetRobotState(std::shared_ptr<DrakeRobotStateWithTorque> robot_state) {
    residual_args_lock_.lock();
    input_args_.robot_state = robot_state;
    residual_args_lock_.unlock();
    this->new_state_available_ = true;
  }


  // will modify the ResidualDetectorState
  // ResidualArgs
  void ResidualDetector::ResidualDetectorUpdate(ResidualDetectorState &residual_detector_state,
                                                const ResidualDetectorInputArgs args) {

    // declare the namespaces that I want to use
    using namespace drake;
    using namespace math;
    using namespace Eigen;

    const auto & q = args.robot_state->q;
    const auto & qd = args.robot_state->qd;


//

    auto q_autodiff = initializeAutoDiff(q);
//    auto qd_autodiff = qd_zero.cast<template_type>();

    this->cache_->initialize(q_autodiff);
    this->rigid_body_tree_->doKinematics(*this->cache_, false);

    // get the mass matrix and it's derivative
    auto mass_matrix_autodiff = this->rigid_body_tree_->massMatrix(*this->cache_);
    auto mass_matrix = autoDiffToValueMatrix(mass_matrix_autodiff);
    auto mass_matrix_gradient = autoDiffToGradientMatrix(mass_matrix_autodiff);

    // get the gravity term
    // note that we set velocity to zero by only initializing the KinematicsCache with a
    // position argument
    const RigidBodyTree<template_type>::BodyToWrenchMap no_external_wrenches;
    auto gravity_term_autodiff = this->rigid_body_tree_->dynamicsBiasTerm(*this->cache_, no_external_wrenches, false);
    auto gravity_term = autoDiffToValueMatrix(gravity_term_autodiff);



    // special case to handle uninitialized state
    if (!residual_detector_state.initialized){
      residual_detector_state.t_prev = args.robot_state->t;
      // generalized momentum at time 0
      residual_detector_state.p_0 = mass_matrix * args.robot_state->qd;
      residual_detector_state.initialized = true;
      return;
    }


    int num_positions = args.robot_state->q.size();
    for(int i=0; i < num_positions; i++){
      int row_idx = i*num_positions;
      residual_detector_state.gamma(i) = gravity_term(i) -
          1/2.0*qd.transpose()*mass_matrix_gradient.block(row_idx, 0, num_positions, num_positions)*qd;
    }

    // time delta since previous call
    double dt = args.robot_state->t - residual_detector_state.t_prev;

    residual_detector_state.integral = residual_detector_state.integral
        + (residual_detector_state.gamma - args.robot_state->torque - gravity_term)*dt;

    residual_detector_state.residual = this->residual_detector_config_.residual_gain
                                       *(residual_detector_state.integral + residual_detector_state.residual
                                       - residual_detector_state.p_0);


    residual_detector_state.t_prev = args.robot_state->t;

    bool verbose = false;
    if (verbose) {
          std::cout << "performing residual detector update" << std::endl;
          std::cout << "dt was " << dt << std::endl;
      }
  }


  // wrapper for ResidualDetectorUpdate that does some copying
  void ResidualDetector::ResidualDetectorUpdateWrapper() {
    this->residual_args_lock_.lock();
    // copy the input args to pass into the ResidualDetectorUpdate method
    ResidualDetectorInputArgs args = input_args_;
    this->residual_args_lock_.unlock();

    this->ResidualDetectorUpdate(this->internal_state_, input_args_);
    this->new_state_available_ = false;
  }

  void ResidualDetector::ThreadLoop(){
    bool done = false;
    while (!done){
      while (!this->new_state_available_){
        std::this_thread::yield();
      }
      this->ResidualDetectorUpdateWrapper();
    }
  }
}

