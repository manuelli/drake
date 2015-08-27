classdef AtlasPlanEval < PlanEval
% A PlanEval which includes some Atlas-specific tweaks. Specifically, it
% calls its plans' getQPControllerInput method with an additional argument
% (contact_force_detected)

  properties (Access = protected)
    robot
  end

  methods
    function obj = AtlasPlanEval(r, varargin)
      obj = obj@PlanEval(varargin{:});
      obj.robot = r;
    end

    function qp_input = getQPControllerInput(obj, t, x, contact_force_detected, byte_array)
      if nargin < 4
        contact_force_detected = zeros(obj.robot.getNumBodies(), 1);
      end

      if nargin < 5
        % constructs a default robot_state_t msg with correctly filled in force_torque message 
        state_msg = obj.constructDummyStateMsg(t,x);
        stream = java.io.ByteArrayOutputStream();
        data_output = java.io.DataOutputStream(stream);
        state_msg.encode(data_output);
        byte_array = stream.toByteArray();
      end

      plan = obj.getCurrentPlan(t, x);
      % this is calling getQPControllerInput in the QPLocomotionPlanCPPWrapper
      % need to make it work even if the current function is called only with (t,x), this is what the 
      % dummy controller is doing
      qp_input = plan.getQPControllerInput(t, x, contact_force_detected, byte_array);
    end
  

    function state_msg = constructDummyStateMsg(obj,t,atlas_state)
      state_msg = drc.robot_state_t();
      state_msg.pose = drc.position_3d_t();
      state_msg.pose.translation = drc.vector_3d_t();
      state_msg.pose.rotation = drc.quaternion_t();
      state_msg.twist = drc.twist_t();
      state_msg.twist.linear_velocity = drc.vector_3d_t();
      state_msg.twist.angular_velocity = drc.vector_3d_t();
      state_msg.force_torque = drc.force_torque_t();

      state_msg.utime = t*1000*1000;
      atlas_dofs = length(atlas_state)/2;
      state_msg.num_joints = atlas_dofs - 6;
      joint_names = cell(state_msg.num_joints, 1);
      joint_names(:) = {'dummy_state_msg'};
      state_msg.joint_name = joint_names;
      
      state_msg.pose.translation.x = atlas_state(1);
      state_msg.pose.translation.y = atlas_state(2);
      state_msg.pose.translation.z = atlas_state(3);
      yaw = atlas_state(6);
      yaw = mod(yaw, 2*pi);
      if (yaw > pi)
        yaw = yaw - 2*pi;
      end
      q = rpy2quat([atlas_state(4) atlas_state(5) yaw]);
      state_msg.pose.rotation.w = q(1);
      state_msg.pose.rotation.x = q(2);
      state_msg.pose.rotation.y = q(3);
      state_msg.pose.rotation.z = q(4);

      
      state_msg.twist.linear_velocity.x = atlas_state(atlas_dofs+1);
      state_msg.twist.linear_velocity.y = atlas_state(atlas_dofs+2);
      state_msg.twist.linear_velocity.z = atlas_state(atlas_dofs+3);
      avel = rpydot2angularvel(atlas_state(4:6), atlas_state(atlas_dofs+4:atlas_dofs+6));
      state_msg.twist.angular_velocity.x = avel(1);
      state_msg.twist.angular_velocity.y = avel(2);
      state_msg.twist.angular_velocity.z = avel(3);

      state_msg.joint_position=zeros(1,state_msg.num_joints);
      state_msg.joint_velocity=zeros(1,state_msg.num_joints);
      state_msg.joint_effort=zeros(1,state_msg.num_joints);
      
      atlas_pos = atlas_state(7:atlas_dofs);
      atlas_vel = atlas_state(atlas_dofs+7:end);
    end
  end
end
