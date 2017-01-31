//
// Created by manuelli on 1/30/17.
//

#include <string>

#include "kuka_iiwa_status_to_residual.h"
#include "yaml-cpp/yaml.h"
#include "robotlocomotion/residual_observer_state_t.hpp"


namespace drake{
namespace examples{
namespace ContactParticleFilter{

  KukaIIWAStatusResidualConfig parseConfig(std::string config_filename){
    KukaIIWAStatusResidualConfig config;
    YAML::Node config_yaml = YAML::LoadFile(config_filename);
    config.publish_channel = config_yaml["channels"]["publish"].as<std::string>();
    config.receive_channel = config_yaml["channels"]["receive"].as<std::string>();
    return config;
  }


  KukaIIWAStatusToResidual::KukaIIWAStatusToResidual(KukaIIWAStatusResidualConfig config):config_(config){}

  void KukaIIWAStatusToResidual::SetupSubscribers() {
    this->lcm_handler_.LCMHandle->subscribe(config_.receive_channel, &KukaIIWAStatusToResidual::onIIWAStatus, this);
  }

  void KukaIIWAStatusToResidual::Start() {
    this->lcm_handler_.Start();
  }

  void KukaIIWAStatusToResidual::Initialize() {
    num_joints_ = this->state_.last_msg.num_joints;
    this->joint_names_.resize(num_joints_);
    this->state_.residual = Eigen::VectorXd::Zero(this->num_joints_);

    std::string iiwa_joint_name_prefix = "iiwa_joint_";
    // convention is 1-indexing
    for(int i=1; i<num_joints_+1; i++){
      this->joint_names_[i] = iiwa_joint_name_prefix + std::to_string(i);
    }

    this->initialized_ = true;
  }

  void KukaIIWAStatusToResidual::onIIWAStatus(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                              const drake::lcmt_iiwa_status *msg) {
//    this->state_.last_msg.reset(msg);
    this->state_.last_msg = *msg;
    if (!this->initialized_){
      this->Initialize();
    }
    this->Update();
  }


  // simple for now, just writes the last message to the residual field
  // could later add some filtering here if we want
  void KukaIIWAStatusToResidual::Update(){
    for(int i=0; i<num_joints_; i++){
      this->state_.residual(i) = this->state_.last_msg.joint_torque_external[i];
    }
  }

  void KukaIIWAStatusToResidual::PublishResidual() {
    robotlocomotion::residual_observer_state_t msg;
    msg.utime = this->state_.last_msg.utime;
    msg.residual.resize(num_joints_);
    msg.gravity.resize(num_joints_);
    msg.internal_torque.resize(num_joints_);
    msg.foot_contact_torque.resize(num_joints_);

    for(int i=0; i<num_joints_; i++){
      msg.residual[i] = this->state_.residual(i);
    }

    this->lcm_.publish(this->config_.publish_channel, &msg);
  }


}// ContactParticleFilter
}// examples
}// drake