#include "residual-detector.hpp"
#include <model-client/model-client.hpp>
#include "drake/util/yaml/yamlUtil.h"

#include <ConciseArgs>
#include <stdexcept>
#include <string>


#define PUBLISH_CHANNEL "RESIDUAL_OBSERVER_STATE"
using namespace Eigen;


ResidualDetector::ResidualDetector(std::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
                                   ResidualDetectorConfig residualDetectorConfig, CommandLineOptions commandLineOptions):
    lcm_(lcm_), verbose_(verbose_), newStateAvailable(false), newResidualStateAvailable(false),
    residualDetectorConfig(residualDetectorConfig), commandLineOptions(commandLineOptions){

//  if (urdfFilename=="none"){
//    std::cout << "using default urdf" << std::endl;
//    std::string drcBase = std::string(std::getenv("DRC_BASE"));
//    urdfFilename = drcBase + "/software/models/atlas_v5/model_LR_RR.urdf";
//    drake_model.addRobotFromURDF(urdfFilename);
//    this->contactFilter.addRobotFromURDF(urdfFilename);
//  }
//  else if (urdfFilename.empty()) {
//    do {
//      botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
//    } while(botparam_ == NULL);
//    std::shared_ptr<ModelClient> model = std::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(),0));
//    drake_model.addRobotFromURDFString(model->getURDFString());
//    this->contactFilter.addRobotFromURDFString(model->getURDFString());
//  }
//  else{
//    drake_model.addRobotFromURDF(urdfFilename);
//    this->contactFilter.addRobotFromURDF(urdfFilename);
//  }

  drake_model.addRobotFromURDF(this->residualDetectorConfig.urdfFilename);
  // if you want to use quaternions. Normally it defaults to ROLLPITCHYAW, need to override by passing optional args
  // see RigidBodyManipulatorURDF.cpp file for the syntax
  // drake_model.addRobotFromURDFString(model->getURDFString(), ".", DrakeJoint::QUATERNION)
  drake_model.compile();


  // setup subscribers using LCMHandler
  lcmHandler.LCMHandle->subscribe("EST_ROBOT_STATE", &ResidualDetector::onRobotState, this);
  lcmHandler.LCMHandle->subscribe("FOOT_CONTACT_ESTIMATE", &ResidualDetector::onFootContact, this);

  if (commandLineOptions.useControllerFootForceTorque){
    lcmHandler.LCMHandle->subscribe("CONTROLLER_STATE", &ResidualDetector::onControllerState, this);
  }else{
    lcmHandler.LCMHandle->subscribe("FOOT_FORCE_TORQUE", &ResidualDetector::onFootForceTorque, this);
  }

  lcmHandler.LCMHandle->subscribe("EXTERNAL_FORCE_TORQUE", &ResidualDetector::onExternalForceTorque, this);

  // Initialize the robot property cache
  YAML::Node control_config = YAML::LoadFile(this->residualDetectorConfig.control_config_filename);
  std::ofstream debug_file(this->residualDetectorConfig.control_config_filename + ".debug.yaml");
  RobotPropertyCache robotPropertyCache = parseKinematicTreeMetadata(control_config["kinematic_tree_metadata"],
                                   drake_model);


  //initialize the kinematics cache
  cache = std::shared_ptr<KinematicsCache<AutoDiffFixedMaxSize>>(new KinematicsCache<AutoDiffFixedMaxSize>(drake_model.bodies)); // keep this around
  cacheTypeDouble = std::shared_ptr<KinematicsCache<double>>(new KinematicsCache<double>(drake_model.bodies));





  //need to initialize the state_driver
//  std::cout << "this should only be printed once" << std::endl;
//  std::vector<std::string> state_coordinate_nadrakemes;

  // build this up manually for now, hacky solution
  this->nq = drake_model.num_positions;
  this->nv = drake_model.num_velocities;

  std::cout << "robot num velocities  = " + this->nv << std::endl;

  for(int i = 0; i < this->nq; i++){
    std::string joint_name = drake_model.getStateName(i);
    this->state_coordinate_names.push_back(joint_name);
  }

  state_driver.reset(new RobotStateDriver(this->state_coordinate_names));


  // this->foot_body_ids[Side::LEFT] = drake_model.findLinkId("l_foot");
  // this->foot_body_ids[Side::RIGHT] = drake_model.findLinkId("r_foot");

  // figure out the left and right FT frame ids

  // either it is a frame or it is a link
  int frameId;
  std::shared_ptr<RigidBodyFrame> leftFTFrame = drake_model.findFrame(this->residualDetectorConfig.leftFootFTFrameName);

  // in this case the frame wasn't found, so it must be a link
  if (leftFTFrame == nullptr){
    frameId = drake_model.findLinkId(this->residualDetectorConfig.leftFootFTFrameName);
  }
  else{
    frameId = leftFTFrame->frame_index;
  }
  this->foot_force_torque_frame_ids[Side::LEFT] = frameId;


  // do the same for the right
  std::shared_ptr<RigidBodyFrame> rightFTFrame = drake_model.findFrame(this->residualDetectorConfig.rightFootFTFrameName);

  // in this case the frame wasn't found, so it must be a link
  if (leftFTFrame == nullptr){
    frameId = drake_model.findLinkId(this->residualDetectorConfig.rightFootFTFrameName);
  }
  else{
    frameId = rightFTFrame->frame_index;
  }

  this->foot_force_torque_frame_ids[Side::RIGHT] = frameId;

  footNames[this->residualDetectorConfig.leftFootName] = Side::LEFT;
  footNames[this->residualDetectorConfig.rightFootName] = Side::RIGHT;

  ForceTorqueMeasurement ftMeasurement;
  ftMeasurement.wrench = Eigen::Matrix<double, 6, 1>::Zero();

  ftMeasurement.frame_idx = drake_model.findLinkId(this->residualDetectorConfig.leftFootName);
  foot_ft_meas_6_axis_zeros[Side::LEFT] = ftMeasurement;

  ftMeasurement.frame_idx = drake_model.findLinkId(this->residualDetectorConfig.rightFootName);
  foot_ft_meas_6_axis_zeros[Side::RIGHT] = ftMeasurement;

  this->args.b_contact_force[Side::LEFT] = false;
  this->args.b_contact_force[Side::RIGHT] = false;


  // initialize the foot contact driver
  // what is this foot_contact_driver even doing???
  foot_contact_driver.reset(new FootContactDriver(robotPropertyCache));

  // initialize the residual_state
  residual_state.t_prev = 0;
  residual_state.r = VectorXd::Zero(nq);
  residual_state.integral = VectorXd::Zero(nq);
  residual_state.gamma = VectorXd::Zero(nq);
  residual_state.p_0 = VectorXd::Zero(nq);
  residual_state.running = false;

  //DEBUGGING
  residual_state.gravity = VectorXd::Zero(nq);
  residual_state.torque = VectorXd::Zero(nq);
  residual_state.foot_contact_joint_torque = VectorXd::Zero(nq);


  this->residualGain = this->residualDetectorConfig.residualGain;
  this->publishChannel = PUBLISH_CHANNEL;

  if (this->commandLineOptions.publishOnDebugChannel){
    this->publishChannel += "_DEBUG";
  }

  this->residualGainVector = residualGain*VectorXd::Ones(this->nq);

  this->foot_FT_6_axis_available = false;


  //TEST
  t_prev = 0;
  return;

}


void ResidualDetector::onRobotState(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                    const bot_core::robot_state_t *msg) {

  if (this->verbose_) {
    std::cout << "got a robot state message" << std::endl;
  }

  std::shared_ptr<DrakeRobotStateWithTorque> state(new DrakeRobotStateWithTorque);
  int nq = this->drake_model.num_positions;
  int nv = this->drake_model.num_velocities;
  state->q = VectorXd::Zero(nq);
  state->qd = VectorXd::Zero(nv);
  state->torque = VectorXd::Zero(nq);

  this->state_driver->decodeWithTorque(msg, state.get());

  // acquire a lock and write to the shared_ptr for robot_state_
  this->pointerMutex.lock();

  this->args.robot_state = state;

  const bot_core::force_torque_t& force_torque = msg->force_torque;

  this->args.foot_force_torque_measurement[Side::LEFT].frame_idx = this->foot_force_torque_frame_ids[Side::LEFT];
  this->args.foot_force_torque_measurement[Side::LEFT].wrench << force_torque.l_foot_torque_x, force_torque.l_foot_torque_y, 0.0, 0.0, 0.0, force_torque.l_foot_force_z;

  this->args.foot_force_torque_measurement[Side::RIGHT].frame_idx = this->foot_force_torque_frame_ids[Side::RIGHT];
  this->args.foot_force_torque_measurement[Side::RIGHT].wrench << force_torque.r_foot_torque_x, force_torque.r_foot_torque_y, 0.0, 0.0, 0.0, force_torque.r_foot_force_z;


  this->pointerMutex.unlock();
  this->newStateAvailable = true;


}

void ResidualDetector::onFootContact(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                     const drc::foot_contact_estimate_t *msg) {

  this->pointerMutex.lock();
  this->args.b_contact_force[Side::LEFT] = msg->left_contact > 0.5;
  this->args.b_contact_force[Side::RIGHT] = msg->right_contact > 0.5;
  this->pointerMutex.unlock();
}

void ResidualDetector::onFootForceTorque(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                         const drc::foot_force_torque_t *msg) {

  this->pointerMutex.lock();
  this->foot_FT_6_axis_available = true;

  this->args.foot_ft_meas_6_axis[Side::LEFT].frame_idx = this->foot_force_torque_frame_ids[Side::LEFT];
  this->args.foot_ft_meas_6_axis[Side::LEFT].wrench << msg->l_foot_torque[0], msg->l_foot_torque[1], msg->l_foot_torque[2], msg->l_foot_force[0], msg->l_foot_force[1], msg->l_foot_force[2];

  this->args.foot_ft_meas_6_axis[Side::RIGHT].frame_idx = this->foot_force_torque_frame_ids[Side::RIGHT];
  this->args.foot_ft_meas_6_axis[Side::RIGHT].wrench << msg->r_foot_torque[0], msg->r_foot_torque[1], msg->r_foot_torque[2], msg->r_foot_force[0], msg->r_foot_force[1], msg->r_foot_force[2];
  this->pointerMutex.unlock();
}


void ResidualDetector::onControllerState(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                         const drc::controller_state_t *msg) {


//  std::cout << "got controller state msg " << std::endl;
//
//  std::cout << "footNames.count(leftFoot)" << this->footNames.count("leftFoot") << std::endl;

  std::string body_name;
  std::map<Side, ForceTorqueMeasurement> foot_ft_meas_6_axis_temp = this->foot_ft_meas_6_axis_zeros; // should be a copy constructor here
  for (auto & contact_output: msg->contact_output){
//    std::cout << "body name " << contact_output.body_name << std::endl;
    if(this->footNames.count(contact_output.body_name) > 0){



//      std::cout << "got controller state FT for " << contact_output.body_name << std::endl;
//      std::vector<double> wrenchVector(contact_output.wrench.begin(), contact_output.wrench.end());
      Eigen::Matrix<double, 6, 1> wrenchMatrix;
      wrenchMatrix << contact_output.wrench[0], contact_output.wrench[1], contact_output.wrench[2],
          contact_output.wrench[3], contact_output.wrench[4], contact_output.wrench[5];
      Side footSide = this->footNames.at(contact_output.body_name);
      foot_ft_meas_6_axis_temp[footSide].wrench = wrenchMatrix;
    }
  }

  // copy it over to the shared data location
  this->pointerMutex.lock();
  this->foot_FT_6_axis_available = true;
  this->args.foot_ft_meas_6_axis = foot_ft_meas_6_axis_temp;
  this->pointerMutex.unlock();

}

void ResidualDetector::onExternalForceTorque(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                           const drake::lcmt_external_force_torque* msg){
  this->pointerMutex.lock();
  this->linksWithExternalForce.clear();
  for (int i=0; i<msg->num_external_forces; i++){
    this->linksWithExternalForce.push_back(msg->body_names[i]);
  }
  this->pointerMutex.unlock();
}


void ResidualDetector::updateResidualState() {
  using namespace Drake;

  // acquire a lock and copy data to local variables
  // need to really understand what data is being copied here, and what is a reference
  this->pointerMutex.lock();
  // we create a new DrakeRobotState every time we get a state message so this should be ok
  std::shared_ptr<DrakeRobotStateWithTorque> robot_state = this->args.robot_state;

  std::map<Side, ForceTorqueMeasurement> foot_force_torque_measurement;
  // if we have a 6-axis FT measurement then use it
  if (this->foot_FT_6_axis_available){
    foot_force_torque_measurement = this->args.foot_ft_meas_6_axis;
  }
  else{
    foot_force_torque_measurement  = this->args.foot_force_torque_measurement;
  }
  //NOTE: the above are deep copies, since none of the things being copied are/contain pointers (as far as I know).
  const auto b_contact_force = this->args.b_contact_force;
  this->newStateAvailable = false;
  this->pointerMutex.unlock();


  double t = robot_state->t;
  double dt = t - this->residual_state.t_prev;

  // compute H for current state (q,qd)
//  KinematicsCache<double> cache = this->drake_model.doKinematics(robot_state->q, robot_state->qd, true);
//  typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize; // this is now declaered in residual-detector.h file



  Matrix<AutoDiffFixedMaxSize, Eigen::Dynamic, 1> q_autodiff(robot_state->q.size());
  initializeAutoDiff(robot_state->q, q_autodiff);
  cache->initialize(q_autodiff);
  drake_model.doKinematics(*cache);
  auto H_autodiff = drake_model.massMatrix(*cache);
  auto H = autoDiffToValueMatrix(H_autodiff);
  auto H_grad = autoDiffToGradientMatrix(H_autodiff);

  MatrixXd footJacobian; // should be 3 x nv
  Vector3d points(0,0,0); //point ifs at origin of foot frame

  //Torques due to foot contact forces: Set to zero by default
  std::map<Side, VectorXd> foot_ft_to_joint_torques;
  foot_ft_to_joint_torques[Side::LEFT] = VectorXd::Zero(this->nv);
  foot_ft_to_joint_torques[Side::RIGHT] = VectorXd::Zero(this->nv);

  for (const auto& side_force: foot_force_torque_measurement){
    const Side& side = side_force.first;
    int frame_or_body_id = side_force.second.frame_idx; // the frame at which that ForceTorque is measured

    Vector6d wrench = side_force.second.wrench;
    // will be same as frame_or_body_id if we are using measured force, otherwise will be expressed in world if coming
    // from CONTROLLER_STATE
    int expressed_in_frame_or_body_id;
    if(this->commandLineOptions.useControllerFootForceTorque){
      expressed_in_frame_or_body_id = frame_or_body_id; // this is world

      // need to rotate the wrench back to body frame
      // need to be careful with autodiff here
      auto worldToFootTransform_autodiff = this->drake_model.relativeTransform(*cache, frame_or_body_id, 0);
      auto rotationMatrix = autoDiffToValueMatrix(worldToFootTransform_autodiff.linear());
      Vector6d wrenchInBodyFrame = Vector6d::Zero();

      wrenchInBodyFrame.head<3>() = rotationMatrix*wrench.head<3>();
      wrenchInBodyFrame.tail<3>() = rotationMatrix*wrench.tail<3>();

      wrench = wrenchInBodyFrame;
    }else{
      expressed_in_frame_or_body_id = frame_or_body_id;
    }

    if (this->verbose_){
      if (b_contact_force.at(side)){
        std::cout << side.toString() << " foot in contact" << std::endl;
      }
      else{
        std::cout << side.toString() << " foot NOT in contact" << std::endl;
      }
    }
    // check if this body is actually in contact or not
    // if not set the contact torques coming from the feet to zero.
    // ignore this for
//    if (!b_contact_force.at(side)){
//      foot_ft_to_joint_torques[side] = VectorXd::Zero(this->nq);
//      continue;
//    }

    // Compute the joint torques resulting from the foot contact
    if (this->residualDetectorConfig.useFootForceTorque){
//      std::cout << "using Foot FT" << std::endl;
      std::vector<int> v_indices;
      auto footJacobian_autodiff = this->drake_model.geometricJacobian(*cache, 0, frame_or_body_id, expressed_in_frame_or_body_id, true, &v_indices);
      footJacobian = autoDiffToValueMatrix(footJacobian_autodiff);
      const Vector6d &wrench = side_force.second.wrench;
      VectorXd joint_torque_at_v_indices = footJacobian.transpose() * wrench;
      VectorXd joint_torque = VectorXd::Zero(this->nq);

      // convert them to a full sized joint_torque vector
      for (int i = 0; i < v_indices.size(); i++){
        joint_torque(v_indices[i]) = joint_torque_at_v_indices(i);
      }

      foot_ft_to_joint_torques[side_force.first] = joint_torque;

      if(this->verbose_){
        std::cout << "v_indices.size()" << v_indices.size() << std::endl;
        std::cout << "footJacobian.cols() " << footJacobian.cols() << std::endl;
        std::cout << "footJacobian.rows() " << footJacobian.rows() << std::endl;
        std::cout << "body_id " << frame_or_body_id << std::endl;
        std::cout << "expressed in " << expressed_in_frame_or_body_id << std::endl;
        // std::cout << "body_id lookup " << drake_model.findLinkId("l_foot") << std::endl;
      }

    }
    else{
      std::cout << "told not to use foot forces, setting foot_ft_to_joint_torques to zero" << std::endl;
      foot_ft_to_joint_torques[Side::LEFT] = VectorXd::Zero(this->nq);
      foot_ft_to_joint_torques[Side::RIGHT] = VectorXd::Zero(this->nq);
    }

    if (this->verbose_){
//      std::cout << side.toString() << " foot force is " << std::endl;
//      std::cout << force << std::endl;

      std::cout << "z force passed through Jacobian is " << foot_ft_to_joint_torques[side](2) << std::endl;
    }
  }

  //compute the gravitational term in manipulator equations, hack by calling doKinematics with zero velocity
  VectorXd qd_zero = VectorXd::Zero(this->nq);
  cacheTypeDouble->initialize(robot_state->q, qd_zero);
  this->drake_model.doKinematics(*cacheTypeDouble, false);


  eigen_aligned_unordered_map<RigidBody const*, Matrix<double, TWIST_SIZE, 1>>
      f_ext; // this is just a dummy force, doesn't do anything
  VectorXd gravity = this->drake_model.dynamicsBiasTerm(*cacheTypeDouble, f_ext, false);

  // computes a vector whose i^th entry is g_i(q) - 1/2*qd^T*dH/dq_i*qd
  VectorXd alpha = VectorXd::Zero(this->nq);
  for (int i=0; i<this->nq; i++){
    int row_idx = i*this->nq;
    alpha(i) = gravity(i) - 1/2*robot_state->qd.transpose()*H_grad.block(row_idx,0,this->nq, this->nq)*robot_state->qd;
  }


  // this is the generalized momentum
  VectorXd p_momentum = H*robot_state->qd;

  //Special case for the first time we enter the loop
  //if this is the first time we are entering the loop, then set p_0, and return. Residual stays at 0
  if (!this->residual_state.running){
    std::cout << "started residual detector updates" << std::endl;
    this->residual_state.running = true;
    this->residual_state.p_0 = p_momentum;
    return;
  }



  // UPDATE STEP
  // First compute new state for residual, only new information this uses is dt, and p_momentum
  VectorXd integral_new = this->residual_state.integral + dt*this->residual_state.gamma;
  VectorXd r_new = this->residualGain*(p_momentum - this->residual_state.p_0 + integral_new);


  // update the residual state
  this->residual_state.r = r_new;
  this->residual_state.integral = integral_new;
  this->residual_state.gamma = alpha - robot_state->torque - this->residual_state.r -
                               foot_ft_to_joint_torques[Side::LEFT] - foot_ft_to_joint_torques[Side::RIGHT]; // this is the integrand;
  this->residual_state.t_prev = t;
  this->newResidualStateAvailable = true;
  this->newResidualStateAvailableForActiveLink = true;

  //DEBUGGING
  this->residual_state.gravity = alpha;
  this->residual_state.torque = robot_state->torque;
  this->residual_state.foot_contact_joint_torque = foot_ft_to_joint_torques[Side::LEFT] + foot_ft_to_joint_torques[Side::RIGHT];


//
//  if (this->verbose_){
//    double Hz = 1/dt;
//    std::cout << "residual update running at " << Hz << std::endl;
////    std::cout << "torque " << this->residual_state.torque << std::endl;
//
//    std::cout << "gamma " << this->residual_state.gamma << std::endl;
//    std::cout << "alpha " << alpha << std::endl;
//    std::cout << "residual " << this->residual_state.r << std::endl;
//    std::cout << "integral_new " << integral_new << std::endl;
//    std::cout << "dt " << dt << std::endl;
//  }

  //publish over LCM
  this->publishResidualState(this->publishChannel, this->residual_state);
}

void ResidualDetector::publishResidualState(std::string publishChannel, const ResidualDetectorState &residual_state){


  // only needs to lock to get the current time
  this->pointerMutex.lock();
  this->residual_state_msg.utime = static_cast<int64_t> (this->residual_state.t_prev * 1e6);
  this->pointerMutex.unlock();

  this->residual_state_msg.num_joints = (int16_t) this->nq;
  this->residual_state_msg.joint_name = this->state_coordinate_names;
  this->residual_state_msg.residual.resize(this->nq);
  this->residual_state_msg.gravity.resize(this->nq);
  this->residual_state_msg.internal_torque.resize(this->nq);
  this->residual_state_msg.foot_contact_torque.resize(this->nq);

  for (int i=0; i < this->nq; i++){
    this->residual_state_msg.residual[i] = (float) residual_state.r(i);

    //DEBUGGING
    this->residual_state_msg.gravity[i] = (float) residual_state.gravity(i);
    this->residual_state_msg.internal_torque[i] = (float) residual_state.torque(i);
    this->residual_state_msg.foot_contact_torque[i] = (float) residual_state.foot_contact_joint_torque(i);
  }

  if (this->verbose_){
    std::cout << "base_z residual is " << residual_state.r(2) << std::endl;
    std::cout << "base_z residual in message is " << this->residual_state_msg.residual[2] << std::endl;
  }

  this->lcm_->publish(publishChannel, &residual_state_msg);
}


void ResidualDetector::residualThreadLoop() {
  bool done = false;

  while (!done){
    while(!this->newStateAvailable){
      std::this_thread::yield();
    }
    this->updateResidualState();
  }
}

void ResidualDetector::start(){

  std::cout << "starting LCMHandler loop" << std::endl;
  this->lcmHandler.Start();
  
  std::cout << "starting residual thread loop" << std::endl;
  this->residualThreadLoop();
}

// is this actually used anywhere???
std::vector<std::vector<std::string>> parseCSVFile(std::string filename){
  using namespace std;
  ifstream file (filename);
  std::vector<std::vector<std::string>> result;
  if (!file.is_open()){
    std::cout << "couldn't find the specified file, is it on the path?" << std::endl;
    return result;
  }

  while (!file.eof()){

//    std::cout << "attempting to read a new line" << std::endl;
    //go through every line
    string line;
    string tmp_string;
    vector<string> tmp;
    size_t pos = string::npos;
    getline(file, line);



    while( (pos=line.find_first_of(",")) != string::npos){
//      std::cout << "the line I just read in is " << std::endl;
//      std::cout << line << std::endl;

//      std::cout << "found a comma at position " << pos << endl;
      // extract the component without the ","
      tmp.push_back(line.substr(0,pos));
      tmp_string = line.substr(0,pos);

//      std::cout << "the string I found is =  " << tmp_string << std::endl;

      //erase the val including the ","
      line.erase(0,pos+1);
//      std::cout << "after erasing the line is" << std::endl;
//      std::cout << line << std::endl;
//      std::cout << "made it through the loop" << std::endl;
    }
    if (tmp.size() > 0){
      result.push_back(tmp);
    }


  }

  return result;
}

ResidualDetectorConfig parseConfig(std::string filename){
  ResidualDetectorConfig config;
  YAML::Node configYAML = YAML::LoadFile(filename);
  YAML::Node robot_data = configYAML["robot_data"];

  std::string drcBase = std::getenv("DRC_BASE");
  // parse robot data
  config.urdfFilename = drcBase + robot_data["urdf"].as<std::string>();
  config.robotType = robot_data["robot_type"].as<std::string>();
  config.control_config_filename = drcBase + robot_data["control_config_filename"].as<std::string>();

  config.leftFootName = robot_data["leftFootName"].as<std::string>();
  config.rightFootName = robot_data["rightFootName"].as<std::string>();
  config.leftFootFTFrameName = robot_data["leftFootFTFrameName"].as<std::string>();
  config.rightFootFTFrameName = robot_data["rightFootFTFrameName"].as<std::string>();


  // parse residual detector specific stuff
  YAML::Node residual_detector = configYAML["residual_detector"];
  config.useFootForceTorque = residual_detector["useFootForceTorque"].as<bool>();
  config.residualGain = residual_detector["gain"].as<double>();
  return config;
}


int main( int argc, char* argv[]){


  // setup the structure to parse the command line options
  ConciseArgs parser(argc, argv);
  CommandLineOptions commandLineOptions;
  commandLineOptions.useControllerFootForceTorque = false;
  commandLineOptions.publishOnDebugChannel = false;
  bool atlas_v5 = false;
  bool val = false;
  bool valkyrie_v1 = false;
  bool valkyrie_v2 = false;
  bool isVerbose = false;
  

  parser.add(atlas_v5, "v5", "atlas_v5", "set robot to atlas_v5");
  parser.add(val, "val", "valkyrie", "set robot to valkyrie");
  parser.add(valkyrie_v1, "val1", "valkyrie_v1", "set robot to valkyrie_v1");
  parser.add(valkyrie_v2, "val2", "valkyrie_v2", "set robot to valkyrie_v2");
  parser.add(isVerbose, "verbose");
  parser.add(commandLineOptions.useControllerFootForceTorque, "useControllerForceTorque", "useControllerForceTorque",
  "use the controllers force torque instead of measured");

  parser.add(commandLineOptions.publishOnDebugChannel, "debug", "publishOnDebugChannel",
  "publish on debug channel, adds _DEBUG to channel name");
  parser.parse();

  std::string configFilename; // config from which we will read the options for the residual detector


  std::string drcBase = std::getenv("DRC_BASE");

  if (atlas_v5){
    std::cout << "using atlas_v5 robot " << std::endl;
    configFilename = drcBase + "/software/control/residual_detector/config/residual_detector_config_atlas_v5.yaml";
  } else if (val){
    std::cout << "using valkyrie robot" << std::endl;
    configFilename = drcBase + "/software/control/residual_detector/config/residual_detector_config_valkyrie.yaml";
  } else{
    throw std::invalid_argument("currently only support atlas_v5, pass -v5 or val, pass -val");
  }

  // TODO: add support for valkyrie

  // initialize LCM
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr << "Error: lcm is not good()" << std::endl;
  }


  ResidualDetectorConfig residualDetectorConfig = parseConfig(configFilename);
  ResidualDetector residualDetector(lcm, isVerbose, residualDetectorConfig, commandLineOptions);

  residualDetector.start();
  return 0;
}

