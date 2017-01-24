#include "residual_detector.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "residual_detector_lcm_wrapper.h"



int main( int argc, char* argv[]){
  using namespace residual_detector;

  std::string path_to_file_from_drake_root = "/examples/ResidualDetector/config/residual_detector_config_kuka_iiwa.yaml";
  ResidualDetectorConfig residual_detector_config = parseResidualDetectorConfig(path_to_file_from_drake_root);
  std::cout << "finished making the residual detector config" << std::endl;


  ResidualDetector residual_detector(residual_detector_config);

//  std::shared_ptr<RigidBodyTree<double>> rbt_state_decoder = std::make_shared<RigidBodyTree<double>>();
//  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
//      residual_detector_config.urdf_filename,
//      drake::multibody::joints::kFixed,
//      nullptr /* weld to frame */, rbt_state_decoder.get());
//
//  RobotStateDecoder robot_state_decoder = RobotStateDecoder(rbt_state_decoder);

  std::cout << std::endl;
  std::cout << "constructing ResidualDetectorLCMWrapper" << std::endl;
  ResidualDetectorLCMWrapper residual_detector_lcm_wrapper(residual_detector, residual_detector_config);
  residual_detector_lcm_wrapper.setupSubscribersKuka();

  std::cout << "starting LCMHandle Thread Loop With Select" << std::endl;
  residual_detector_lcm_wrapper.start();

  std::thread & lcm_thread_handle = residual_detector_lcm_wrapper.lcm_handler_.ThreadHandle;
  std::cout << "blocking until that thread finishes" << std::endl;

  // main loop of residual detector
  std::thread residual_detector_thread_handle = std::thread(&ResidualDetector::ThreadLoop, &residual_detector);

  lcm_thread_handle.join();
  residual_detector_thread_handle.join();
}