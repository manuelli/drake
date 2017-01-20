//
// Created by manuelli on 11/25/15.
//


#include <mutex>
#include <thread>
#include <chrono>
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "RobotStateDriver.hpp"
#include "FootContactDriver.hpp"
#include "drc_utils/LCMHandler.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"
#include "lcmtypes/drc/foot_force_torque_t.hpp"
#include "lcmtypes/drc/residual_observer_state_t.hpp"
#include "lcmtypes/drc/controller_state_t.hpp"
#include "lcmtypes/drake/lcmt_external_force_torque.hpp"
#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>
#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>

#include "drake/core/Gradient.h"
#include "drake/systems/plants/KinematicsCache.h"



#ifndef CONTROL_RESIDUAL_DETECTOR_H
#define CONTROL_RESIDUAL_DETECTOR_H

#endif //CONTROL_RESIDUAL_DETECTOR_H


// internal state for the residual detector class
struct ResidualDetectorState{
  double t_prev;
  VectorXd r;
  VectorXd integral;
  VectorXd gamma;
  VectorXd p_0;
  bool running;

  // for debugging purposes
  VectorXd gravity;
  VectorXd torque;
  VectorXd foot_contact_joint_torque;

};

// Arguments that are grabbed from LCM messages.
struct ResidualArgs{
  // information that updateResidualState will need
  std::shared_ptr<DrakeRobotStateWithTorque> robot_state;
  std::map<Side, ForceTorqueMeasurement> foot_force_torque_measurement;
  std::map<Side, ForceTorqueMeasurement> foot_ft_meas_6_axis;
  std::map<Side, bool> b_contact_force;
};

struct ResidualDetectorConfig{
  std::string robotType;
  std::string urdfFilename;
  std::string control_config_filename;
  std::string leftFootName;
  std::string rightFootName;
  std::string leftFootFTFrameName;
  std::string rightFootFTFrameName;
  double residualGain;
  bool useFootForceTorque;
};

struct CommandLineOptions{
  bool useControllerFootForceTorque;
  bool publishOnDebugChannel;
};

class ResidualDetector{

public:
  ResidualDetector(std::shared_ptr<lcm::LCM> &lcm_, bool verbose_, ResidualDetectorConfig residualDetectorConfig,
  CommandLineOptions commandLineOptions);
  ~ResidualDetector(){
  }
  void start();
  void residualThreadLoop();
  void kinematicChain(std::string linkName, int body_id =-1);
  void testContactFilterRotationMethod(bool useRandom=false);
  void testQP();
  void contactFilterThreadLoop(std::string filename);
  void computeActiveLinkContactFilter(bool publish, bool useActiveLinkInfo);
  void activeLinkContactFilterThreadLoop();

  RigidBodyTree drake_model;


private:
  std::shared_ptr<lcm::LCM> lcm_;
  LCMHandler lcmHandler;
  bool running_;
  bool verbose_;
  bool newStateAvailable;
  bool newResidualStateAvailable;
  bool newResidualStateAvailableForActiveLink;
  bool foot_FT_6_axis_available;
  int nq;
  int nv;
  double t_prev;
  CommandLineOptions commandLineOptions;
  std::mutex pointerMutex;
  std::vector<std::string> state_coordinate_names;
  std::string publishChannel;
  std::thread residual_thread_handle;

  std::vector<std::string> linksWithExternalForce;

  VectorXd residualGainVector;
  double residualGain;
  ResidualArgs args;
  ResidualDetectorState residual_state;
  ResidualDetectorConfig residualDetectorConfig;
//  ResidualDetectorState residual_state_w_forces;
  drc::residual_observer_state_t residual_state_msg;

  std::map<Side, ForceTorqueMeasurement> foot_ft_meas_6_axis_zeros; // all zeros
  std::map<Side, int> foot_body_ids;
  std::map<Side, int> foot_force_torque_frame_ids;
  std::map<std::string, Side> footNames;
  typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize;
  std::shared_ptr<KinematicsCache<AutoDiffFixedMaxSize>> cache;
  std::shared_ptr<KinematicsCache<double>> cacheTypeDouble;

  std::shared_ptr<RobotStateDriver> state_driver;
  std::shared_ptr<FootContactDriver> foot_contact_driver;

  // message handlers
  void onRobotState(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::robot_state_t* msg);
  void onFootContact(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::foot_contact_estimate_t* msg);
  void onFootForceTorque(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::foot_force_torque_t* msg);
  void onExternalForceTorque(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drake::lcmt_external_force_torque* msg);
  void onControllerState(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::controller_state_t* msg);

  // All the actual computation happens in this method
  void updateResidualState();
  void publishResidualState(std::string publish_channel, const ResidualDetectorState &);
};


// a helper method for parsing the config
ResidualDetectorConfig parseConfig(std::string configFile);
