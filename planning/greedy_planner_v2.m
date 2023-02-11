classdef greedy_planner_v2 < handle
    properties (SetAccess = public, GetAccess = public)
        num_robot;
        robot_idx;
        actions;    % discrete action set
        n_actions;
        
        smm;    %sensor motion model
        som;    %sensor observation model
    end
    methods
        function this = greedy_planner_v2(num_robot, robot_idx, actions, samp, r_sense, fov)
            this.actions = actions;
            this.n_actions = length(actions);
            this.robot_idx = robot_idx;
            this.num_robot = num_robot;
            
            this.smm.samp = samp;
            this.smm.f = @(x,u) point_mass_motion_model(x(1:3,:),u,this.smm.samp);

            this.som.r_sense = r_sense;
            this.som.fov = fov;
        end
    
        function [greedy_action_idx, greedy_next_state] = greedy_action(this, t, x, y, prev_robot_states, r_sense, fov)
            % this function selects the next action using Greedy
            % x: this robot's state, 3 x 1
            % y: all targets' states, 2 x num_tg            
            % output: action for t+1
            
            % for each robot
            num_tg = size(y, 1);
            future_x = this.smm.f(x, this.actions);
            n_x = size(future_x, 2); % number of potential next state
            temp_obj = zeros(n_x, 1);

            for i = 1 : n_x
                temp_x = future_x(:, i);
                temp_robot_states = [prev_robot_states temp_x];
                temp_obj(i) = objective_function(temp_robot_states, y, r_sense, fov);
            end
            [max_obj, greedy_action_idx] = max(temp_obj);
            if sum(temp_obj == max_obj) > 1
                %warning('same greedy value')
                prob_v = 1 / sum(temp_obj == max_obj);
                prob = zeros(this.n_actions, 1);
                prob(temp_obj == max_obj) = prob_v;
                greedy_action_idx = discretesample(prob, 1);
            end
            greedy_next_state = future_x(:, greedy_action_idx);
        end
    end
end