classdef KinematicPlan < QPControllerPlan

  properties
    supports;
    support_times;
    support_names; % cell array of all supports used in the plan
    c_pts;
    linkId;
    qtraj;
    support_logic_type;
    breaking_contact_time_threshold = 0.25;
  end

  methods

    function obj = KinematicPlan(data)
      
      % data_supports should be a cell array of cell arrays
      support_data = data.supports;
      obj = obj.initializeSupports(support_data);
      obj.support_times = data.support_times;
      obj.c_pts = data.c_pts;
      obj.linkId = data.linkId;
      obj.qtraj = data.qtraj;
      obj.duration = obj.qtraj.tspan(2) - obj.qtraj.tspan(1);

      % choose the type of support logic to implement
      if isfield(data,'support_logic_type')
        obj.support_logic_type = data.support_logic_type;
      else
        obj.support_logic_type = 'require_support';
      end
    end

    function obj = initializeSupports(obj,support_data)
      % list of all the supports utilized in the plan
      support_names = containers.Map;
      for j = 1:numel(support_data)
        for k = 1:numel(support_data{j})
          name = support_data{j}{k};
          if ~isKey(support_names,name)
            support_names(name) = true;
          end
        end
      end
      obj.support_names = keys(support_names);

      % cell array of container maps
      supports = {};
      for j = 1:numel(support_data)
        supp = containers.Map;
        for k = 1:numel(support_data{j})
          name = support_data{j}{k};
          supp(name) = struct('in_support',1,'breaking_contact',0);
        end

        % now deal with the contacts that the plan says aren't active
        non_active_contacts = setdiff(keys(support_names),keys(supp));

        for k = 1:numel(non_active_contacts)
          name = non_active_contacts{k};
          supp(name) = struct('in_support',0,'breaking_contact',0);

          % check to see if it is breaking contact, only do this for j > 1
          % this means it was in support at the previous time step
          if (j>1) && supports{j-1}(name).in_support
            temp = supp(name);
            temp.breaking_contact = 1;
            supp(name) = temp;
          end
        end

        % assign the containers.Map supp to the correct entry of obj.supports
        supports{j} = supp;
      end
      obj.supports = supports;

    end

    function qp_input = getQPControllerInput(obj,t_global,x)
      import atlasControllers.*
      % convert global time to plan time
      t = t_global - obj.start_time;
      qp_input = QPInput3D();
      qp_input = obj.setSupportData(t,x,qp_input);
      %qp_input = obj.setDefaultCosts(qp_input);

      % not sure what support_data.param_set_name should be, by default it is set to 'walking'
    end

    % Implements three types of support logic depending on whether it is active in the plan, whether it is just 
    % breaking support, or any remaining situation
    function qp_input = setSupportData(obj,t,x,qp_input)
      % this t is already in plan time

      support_data = qp_input.support_data;
      I = find(obj.support_times < t);
      idx = I(end);
      supp = obj.supports{idx}; % supports plan thinks are active

      % now construct the support data for each possible contact
      % obj.support_names is exactly keys(supp)
      for j = 1:numel(obj.support_names)
        name = obj.support_names{j};
        support_data(j).body_id = obj.linkId(name);
        support_data(j).contact_pts = obj.c_pts(name);
        support_data(j).mu = 1;
        support_data(j).contact_surfaces = 0;

        % apply the correct contact logic
        if supp(name).in_support
          support_data(j).support_logic_map = obj.support_logic_maps.(obj.support_logic_type);
        % if we are just breaking support with that contact then use 'prevent_support'
        elseif supp(name).breaking_contact && ((t - obj.support_times(idx)) < obj.breaking_contact_time_threshold)
          support_data(j).support_logic_map = obj.support_logic_maps.prevent_support;
        else
          % in this case it isn't in contact but it hasn't just broken contact so use kinematics/force to 
          % determine whether it is in contact
          support_data(j).support_logic_map = obj.support_logic_maps.kinematic_or_sensed;
        end    

      end
      qp_input.support_data = support_data;
    end
    
  end

end