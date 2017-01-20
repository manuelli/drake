include_directories(${CMAKE_SOURCE_DIR}/src)
#include_directories(${GUROBI_HOME}/lib) # super hacky, remove this later

# (Lucas Manuelli) RobotStateDriver library needed for residual detector
add_library(RobotStateDriver RobotStateDriver.cpp)
target_link_libraries(RobotStateDriver 
    drakeGeometryUtil
    lcmtypes_bot2-core-cpp)
drake_install_libraries(RobotStateDriver)
drake_install_headers(RobotStateDriver.hpp)

# pods_find_pkg_config(gurobi)
#add_executable(drc-residual-detector residual-detector.hpp residual-detector.cpp)
#target_link_libraries(drc-residual-detector
#	RobotStateDriver
#	FootContactDriver
#	drakeJoints
#	drakeYAMLUtil
#	drakeRBM
#	drakeSide)

#pods_use_pkg_config_packages(drc-residual-detector
#	lcmtypes_drc_lcmtypes
#	lcm model-client drake-rbm drake-side eigen3 yaml-cpp)
#drake_install_executables(drc-residual-detector)






