//
// Created by manuelli on 1/20/17.
//

#include "residual_detector_lcm_wrapper.h"
#include "drake/multibody/parsers/urdf_parser.h"


namespace residual_detector{

  ResidualDetectorLCMWrapper::ResidualDetectorLCMWrapper(ResidualDetector &residual_detector,
                                                         ResidualDetectorConfig residual_detector_config):
  residual_detector_(residual_detector), residual_detector_config_(residual_detector_config){

    std::shared_ptr<RigidBodyTree<double>> rbt_state_decoder = std::make_shared<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        residual_detector_config.urdf_filename,
        drake::multibody::joints::kFixed,
        nullptr /* weld to frame */, rbt_state_decoder.get());

    robot_state_decoder_ = RobotStateDecoder(rbt_state_decoder);

  };


  void ResidualDetectorLCMWrapper::start(){
    this->lcm_handler_.Start();
  };


  void ResidualDetectorLCMWrapper::setupSubscribersKuka() {
    lcm::Subscription* sub;
    sub = this->lcm_handler_.LCMHandle->subscribe("EST_ROBOT_STATE",
                                                  &ResidualDetectorLCMWrapper::onRobotState, this);

    sub->setQueueCapacity(1);
  };

  void ResidualDetectorLCMWrapper::onRobotState(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                                const bot_core::robot_state_t *msg) {

//    std::cout << "got a robot state message " << std::endl;
    std::shared_ptr<DrakeRobotStateWithTorque> robot_state = this->robot_state_decoder_.decodeRobotStateMsg(msg);

    this->residual_detector_.SetRobotState(robot_state);
  }

}