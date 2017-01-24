import numpy
import colorsys

num_joints = 2

addPlot(timeWindow=15, yLimits=[-3.14, 3.14])
addSignals('IIWA_STATUS', msg.utime, msg.joint_torque_measured, range(num_joints))
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_effort, range(num_joints))

addPlot(timeWindow=15, yLimits=[-3.14, 3.14])
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_velocity, range(num_joints))


addPlot(timeWindow=15, yLimits=[-3.14, 3.14])
addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, range(num_joints))

