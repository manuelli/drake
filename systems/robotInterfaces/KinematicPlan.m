classdef KinematicPlan < QPControllerPlan

  import atlasControllers.*

  properties
    supports;
    support_times;
    c_pts;
    linkId;
    qtraj;
    support_logic_type;
    breaking_contact_time_threshold = 0.25;
  end

  methods

    function obj = KinematicPlan(qtraj,data)
      obj = obj.initializeSupports(obj,data.supports);
      obj.support_times = data.support_times;
      obj.c_pts = data.c_pts;
      obj.linkId = data.linkId;
      obj.qtraj = qtraj;

      % choose the type of support logic to implement
      if isfield(data,'support_logic_type')
        obj.support_logic_type = data.support_logic_type;
      else
        obj.support_logic_type = 'require_support';
      end
    end

    function obj = initializeSupports(obj,data.supports)
      % list of all the supports utilized in the plan
      support_names = containers.Map;
      for j = numel(data.supports)
        for k = numel(data(j).supports)
          name = data(j).supports{k};
          if ~iskey(support_names,name)
            support_names(name) = true;
          end
        end
      end

      % cell array of container maps
      supports = {};
      for j = 1:numel(data.supports)
        supp = containers.Map;
        for k = 1:numel(data(j).supports)
          name = data(j).supports{k};
          supp(name) = struct('in_support',1,'breaking_contact',0);
        end

        % now deal with the contacts that the plan says aren't active
        non_active_contacts = setdiff(keys(support_names),keys(supp));

        for k = 1:numel(non_active_contacts)
          name = non_active_contacts{k};
          supp(name) = struct('in_support',1,'breaking_contact',0);

          % check to see if it is breaking contact, only do this for j > 1
          % this means it was in support at the previous time step
          if (j>1) && supports{j-1}(name).in_support
            supp(name).breaking_contact = 1;
          end
        end

        % assign the containers.Map supp to the correct entry of obj.supports
        supports{j} = supp;
      end
      obj.supports = supports;

    end

    function qp_input = getQPControllerInput(obj,t_global,x)
      % convert global time to plan time
      t = t_global - obj.start_time;
      qp_input = QPInput3D();
      qp_input = obj.setSupportData(t,x,qp_input);
      qp_input = obj.setDefaultCosts(qp_input);

      % not sure what support_data.param_set_name should be, by default it is set to 'walking'
    end

    % this will be very basic at the moment, need to implement the breaking contact logic as well
    function support_data = setSupportData(obj,t,x,qp_input)
      support_data = qp_input.support_data;
      I = find(obj.support_times < t);
      idx = I(end);
      supp = obj.supp(idx); % supports plan thinks are active

      % now construct the support data for each possible contact
      for j = 1:numel(keys(supp))
        name = keys(supp){j};
        support_data(j).body_id = obj.linkId(name);
        support_data(j).contact_pts = obj.c_pts(name);
        support_data(j).mu = 1;
        support_data(j).contact_surfaces = 0;

        % apply the correct contact logic
        if supp(name).in_support
          support_data(j).support_logic_map = getfield(obj.support_logic_maps,support_logic_type);
        % if we are just breaking support with that contact then use 'prevent_support'
        elseif supp(name).breaking_contact && ((t - obj.support_times(idx)) < obj.breaking_contact_time_threshold)
          support_data(j).support_logic_map = getfield(obj.support_logic_maps,'prevent_support');
        else
          % in this case it isn't in contact but it hasn't just broken contact so use kinematics/force to 
          % determine whether it is in contact
          support_data(j).support_logic_map = getfield(obj.support_logic_maps,'kinematics_or_sensed');
        end    

      end
      qp_input.support_data = support_data;
    end
    
  end

end