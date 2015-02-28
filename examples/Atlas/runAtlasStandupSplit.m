function runAtlasWalking(example_options)
% Example of QPController for standing up with a kinematic plan
%
% @option use_mex
% @option use_bullet
% @option use_angular_momentum
% @options navgoal
% 

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
if ~isfield(example_options,'use_mex'), example_options.use_mex = true; end
if ~isfield(example_options,'use_bullet') example_options.use_bullet = false; end
if ~isfield(example_options,'use_angular_momentum') example_options.use_angular_momentum = false; end
if ~isfield(example_options,'navgoal')
%  navgoal = [2*rand();0.25*randn();0;0;0;0];
  example_options.navgoal = [0.2;0;0;0;0;0];
end
if ~isfield(example_options,'terrain'), example_options.terrain = RigidBodyFlatTerrain; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

path_handle = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/prone']);

% construct robot model
options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.001;
options.terrain = example_options.terrain;
% probably want v5 here
atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_minimal_contact.urdf'];
% r = TimeSteppingRigidBodyManipulator(atlas_urdf,options.dt,options);
r = Atlas(atlas_urdf,options);

% this sets up exactly the collision geometries that we want on the robot
kpt = KinematicPoseTrajectory(r,{});
r = kpt.addSpecifiedCollisionGeometryToRobot({'l_toe','l_knee','l_hand','r_toe','r_knee','r_hand'});
r = r.removeCollisionGroupsExcept({'l_toe','l_knee','l_hand','r_toe','r_knee','r_hand'});
r = kpt.addVisualContactPoints(r);
r = compile(r);

% the variable should be called plan_data, cell array of structs
load('data_one_knee_plan')
% should already have the fields, supports, support_times and qtraj
kinematic_plan_data = plan_data{2};

% populate the remaining fields
kinematic_plan_data.c_pts = kpt.c;
kinematic_plan_data.linkId = kpt.linkId;

qstar = kinematic_plan_data.qtraj.eval(kinematic_plan_data.qtraj.tspan(1));
xstar = [qstar;0*qstar]; % want the robot to start with zero velocity
r = r.setInitialState(xstar);


% set initial state to fixed point
% load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
% if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
% xstar = r.resolveConstraints(xstar);
% r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.01;

nq = getNumPositions(r);

x0 = xstar;

% Find the initial positions of the feet
% R=rotz(example_options.navgoal(6));

% rfoot_navgoal = example_options.navgoal;
% lfoot_navgoal = example_options.navgoal;

% rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
% lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% % Plan footsteps to the goal
% goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
% footstep_plan = r.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', struct('max_num_steps', 1)));

% walking_plan_data = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan);

import atlasControllers.*;

param_sets = atlasParams.getDefaults(r);
control = AtlasPlanlessQPController(r,...
                                    @statelessBodyMotionControlmex,...
                                    param_sets, struct('use_mex', 2));
                                    % fcompare(@statelessBodyMotionControl,@statelessBodyMotionControlmex),...

kinematic_plan = KinematicPlan(kinematic_plan_data);
planeval = AtlasPlanEval(r, kinematic_plan);
% plancontroller = AtlasSplitQPController(r, control, planeval);

% plan_node = AtlasSplitQPController(r, [], planeval);
% control_node = AtlasSplitQPController(r, control, []);
% plancontroller = cascade(plan_node, control_node);
plancontroller = AtlasSplitQPController(r,control,planeval);

sys = feedback(r, plancontroller);
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

% simulate almost the entire length of time
T = kinematic_plan.duration-0.001;

% profile on
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
% profile viewer

v.playback(ytraj, struct('slider', true));

