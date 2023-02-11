classdef greedy_planner_v1 < handle
    properties (SetAccess = public, GetAccess = public)
        num_robot;
        robot_idx;
        actions;    % discrete action set
        n_actions;
        
        smm;    %sensor motion model
        som;    %sensor observation model
    end
    methods
        function this = greedy_planner_v1(num_robot, robot_idx, actions, samp, r_sense, fov)
            this.actions = actions;
            this.n_actions = length(actions);
            this.robot_idx = robot_idx;
            this.num_robot = num_robot;
            
            
            this.smm.samp = samp;
            this.smm.f = @(x,u) dd_motion_model(x(1:3,:),u,this.smm.samp,true);

            this.som.r_sense = r_sense;
            this.som.fov = fov;


        end
    
        function [loss, obj_mat, greedy_action_idx] = greedy_action(this, t, robot_idx, x, y, prev_loss, prev_obj_mat)
            % for each robot
            num_tg = size(y, 1);
            future_x = this.smm.f(x, this.actions);
            n_x = size(future_x, 2);
            cur_reward = zeros(n_x, 1);
            for i = 1 : n_x
                cur_x = future_x(:, i);
                obj_mat = prev_obj_mat;
                for kk = 1 : num_tg
                    cur_y = y(kk, 1:2)';
                    valid = visibility(x, cur_y, this.som.r_sense, this.som.fov);
                    if valid
                        range = norm(cur_x(1:2) - cur_y);
                        obj_mat(kk) = obj_mat(kk) - 1 / (range.^2);
                    end
                end
                cur_reward(i) = reward_function(obj_mat) - (1-prev_loss);
            end
            [max_reward, greedy_action_idx] = max(cur_reward);
            if sum(cur_reward == max_reward) > 1
                %warning('same greedy value')
                prob_v = 1/sum(cur_reward == max_reward);
                prob = zeros(this.n_actions,1);
                prob(cur_reward == max_reward) = prob_v;
                greedy_action_idx = discretesample(prob, 1);
            end
            loss = 1-max_reward;
            

            

        end
    end
        


end