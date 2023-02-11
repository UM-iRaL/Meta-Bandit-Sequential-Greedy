classdef bsg_planner_nx_v1 < handle
    properties (SetAccess = public, GetAccess = public)
        actions;        % discrete action set
        n_actions;      % number of actions
        action_indices; 
        robot_idx;
        num_robot;


        selected_action_index;
        % BSG params
        J; % number of experts
        e;
        beta;
        eta;
        gamma;
        expert_weight;
        action_weight;
        action_prob_dist;
        loss;
        loss_estm;
        
        %
        som;
        tmm;
        smm;
        
    end
    methods
        function this = bsg_planner_nx_v1(num_robot, robot_idx, actions, n_time_step, samp, r_sense, fov, stdev_z)
            this.num_robot = num_robot;
            this.robot_idx = robot_idx;
            this.actions = actions;
            this.n_actions = length(actions);
            this.action_indices = 1:this.n_actions;

            this.selected_action_index = ones(n_time_step, 1);
%             learning_const = 1; % non-adversarial 2v2
            %learning_const = 1; % non-adversarial 2v3
%             learning_const = 1; % non-adversarial 2v4
            %learning_const = 4; % adversarial 2v2
            %learning_const = 1; % adversarial 2v3
            learning_const = 4; %non-adversarial 2v4

            this.J = ceil(log(n_time_step));
            this.e = learning_const*sqrt(log(this.J) / 2 / n_time_step);
            this.beta = 1 / (n_time_step - 1);
            this.eta = zeros(this.J, 1);
            for j = 1 : this.J
                this.eta(j) = learning_const*sqrt(log(this.n_actions*n_time_step) / pow2(j-1) / this.n_actions);
            end
            this.gamma = this.eta / 2;
            this.expert_weight = 1 / this.J * ones(n_time_step, this.J);
            this.action_weight = 1 / this.n_actions * ones(n_time_step, this.J, this.n_actions);
            this.action_prob_dist = zeros(n_time_step, this.n_actions);
            this.loss = zeros(n_time_step, this.n_actions);
            this.loss_estm = zeros(n_time_step, this.n_actions);
                        
            this.som.r_sense = r_sense;
            this.som.fov = fov;
            this.som.stdev_z = stdev_z;

           
            this.smm.samp = samp;
            this.smm.f = @(x,u) point_mass_motion_model(x(1:3,:),u,this.smm.samp);
            
        end
        
        function update_experts(this, t)
            % update weights
            for j = 1:this.J
                %nxt_a_idx = this.next_action_index(t);
                this.loss_estm(t, this.selected_action_index(t)) = this.loss(t, this.selected_action_index(t)) /...
                    (this.action_prob_dist(t, this.selected_action_index(t)) + this.gamma(j));
                v = zeros(this.n_actions,1);
                for i = 1 : this.n_actions
                    v(i) = this.action_weight(t, j, i) * exp(-this.eta(j)*this.loss_estm(t, i));                    
                        %v(i) = this.action_weight(t, j, i) * exp(this.eta(j)*(1-this.loss_estm(t, i)));
                end
                W_t = sum(v);
                this.action_weight(t+1,j,:) = this.beta*W_t/this.n_actions + (1-this.beta)*v;
                    
                this.expert_weight(t+1,j) = this.expert_weight(t, j)*...
                    exp(-this.e*this.loss_estm(t,:)*(reshape(this.action_weight(t, j,:), this.n_actions,[])/norm(squeeze(this.action_weight(t, j,:)),1)));
                
                %this.expert_weight(t+1,j) = this.expert_weight(t, j)*...
                %   exp(this.e*(1-this.loss_estm(t,:))*(reshape(this.action_weight(t, j,:), this.n_actions,[])/norm(squeeze(this.action_weight(t, j,:)),1)));
            end
            % normalize weights by 1-Norm
            for j = 1:this.J
                this.action_weight(t+1, j, :) = this.action_weight(t+1, j, :) / norm(squeeze(this.action_weight(t+1, j, :)), 1);
                if sum(isnan(this.action_weight(t+1, j, :))) > 0
                    warning("Nan value is not valid.");
                end
            end
            this.expert_weight(t+1,:) = this.expert_weight(t+1,:) / norm(squeeze(this.expert_weight(t+1,:)), 1);
        end
                
        function update_action_prob_dist(this, t)
%             if t == 1
%                 this.action_prob_dist(t, :) = 1/this.n_actions*ones(1, this.n_actions);
%             end
            q_t = this.expert_weight(t,:);    % 1 * J
            p_t = this.action_weight(t,:,:);  % J * n_actions
            
            this.action_prob_dist(t, :) = squeeze(q_t) * squeeze(p_t);  % 1 * n_actions
            if sum(isnan(this.action_prob_dist(t, :))) > 0
                warning("nan value is not valid");
            end
            if abs(sum(this.action_prob_dist(t, :)) - 1) > 1e-4
                error("incorrect prob distribution");
            end
        end
    end
end


