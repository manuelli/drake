add_executable(run_residual_detector
    residual_detector_test.cc
    residual_detector.cc
    residual_detector_utils.cc
    residual_detector_lcm_wrapper.cc)

target_link_libraries(run_residual_detector
    Eigen3::Eigen
    drakeCommon
    drakeRBM
    drakeMultibodyParsers
    lcmtypes_bot2-core-cpp
    robotlocomotion-lcmtypes-cpp
    drakeLCMUtil
    drakeMath
    yaml-cpp
    lcm)