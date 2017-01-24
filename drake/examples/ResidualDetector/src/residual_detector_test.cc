#include "residual_detector.h"
#include "residual_detector_lcm_wrapper.h"



int main( int argc, char* argv[]){
  using namespace residual_detector;

  std::string path_to_file_from_drake_root = "/examples/ResidualDetector/config/residual_detector_config_kuka_iiwa_sim.yaml";
  ResidualDetectorConfig residual_detector_config = parseResidualDetectorConfig(path_to_file_from_drake_root);
  std::cout << "finished making the residual detector config" << std::endl;

  ResidualDetector residual_detector(residual_detector_config);


  std::cout << std::endl;
  std::cout << "constructing ResidualDetectorLCMWrapper" << std::endl;
  ResidualDetectorLCMWrapper residual_detector_lcm_wrapper(residual_detector, residual_detector_config);
  residual_detector_lcm_wrapper.setupSubscribersKuka();

  std::cout << "starting LCMHandle Thread Loop With Select" << std::endl;
  residual_detector_lcm_wrapper.start();
  std::thread & lcm_thread_handle = residual_detector_lcm_wrapper.lcm_handler_.ThreadHandle;

  // main loop of residual detector
  std::thread residual_detector_thread_handle = std::thread(&ResidualDetector::ThreadLoop, &residual_detector);

  // block until these threads finish
  lcm_thread_handle.join();
  residual_detector_thread_handle.join();
}