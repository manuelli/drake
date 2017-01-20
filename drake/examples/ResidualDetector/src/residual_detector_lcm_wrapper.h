//
// Created by manuelli on 1/20/17.
//

#ifndef DRAKE_DISTRO_RESIDUAL_DETECTOR_LCM_WRAPPER_H
#define DRAKE_DISTRO_RESIDUAL_DETECTOR_LCM_WRAPPER_H

#include "residual_detector.h"
#include "residual_detector_utils.h"
#include "LCMHandler.h"


namespace residual_detector{
  class ResidualDetectorLCMWrapper {
  public:
    LCMHandler lcm_handler_;

    ResidualDetectorLCMWrapper(ResidualDetector & residual_detector, ResidualDetectorConfig residual_detector_config);
    void start();
    void setupSubscribersKuka();
  private:
    ResidualDetector& residual_detector_;
    ResidualDetectorConfig residual_detector_config_;
    RobotStateDecoder robot_state_decoder_;


    void onRobotState(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                      const bot_core::robot_state_t* msg){

      std::cout << "got a robot state message " << std::endl;
      DrakeRobotStateWithTorque robot_state = this->robot_state_decoder_.decodeRobotStateMsg(msg);
    }
  };
}



#endif //DRAKE_DISTRO_RESIDUAL_DETECTOR_LCM_WRAPPER_H
