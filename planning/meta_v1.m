classdef meta_v1 < handle
    properties (SetAccess = protected, GetAccess = public)
        actions;    % discrete action set
        n_actions;
        action_indices;

        num_expert;
        expert_weight;
        action_prob_dist;

        loss_estm;
        e;
    end

    properties (SetAccess = public, GetAccess = public)
        selected_action_index;
        action_weight;
        loss;
    end

    methods
        function this = meta_v1(actions, num_expert, n_time_step)
            this.actions = actions;
            this.n_actions = length(actions);
            this.action_indices = 1 : this.n_actions;
            this.num_expert = num_expert;
            
            this.loss = zeros(n_time_step, this.n_actions);
            this.loss_estm = zeros(n_time_step, this.n_actions);
            this.action_prob_dist = zeros(n_time_step, this.n_actions);
            this.expert_weight = 1 / num_expert * ones(n_time_step, num_expert);
            this.action_weight = 1 / this.n_actions * zeros(n_time_step, num_expert, this.n_actions);
            this.e = 0.017;
            
        end

        function update_experts(this, t)

            for j = 1 : this.num_expert
                this.loss_estm(t, this.selected_action_index(t)) = this.loss(t, this.selected_action_index(t)) /...
                   (this.action_prob_dist(t, this.selected_action_index(t))); 
                % action weight need to be updated from experts first.
                
                this.expert_weight(t+1,j) = this.expert_weight(t, j)*...
                    exp(-this.e*this.loss_estm(t,:)*(reshape(this.action_weight(t, j,:), this.n_actions,[])/norm(squeeze(this.action_weight(t, j,:)),1)));
            end
            beta = 4/1000;
            W_t = sum(this.expert_weight(t+1, :));
            this.expert_weight(t+1, :) = beta*W_t/this.num_expert + (1 - beta)*this.expert_weight(t+1, :);
            this.expert_weight(t+1,:) = this.expert_weight(t+1,:) / norm(squeeze(this.expert_weight(t+1,:)), 1);
%             if var(this.expert_weight(t+1, :)) > 0.3 %skew enough
%                 this.expert_weight(t+1,:) = (this.expert_weight(t+1,:) + 1);
%                 this.expert_weight(t+1,:) = this.expert_weight(t+1,:) / norm(squeeze(this.expert_weight(t+1,:)), 1);
%             end
        end

        function update_action_prob_dist(this, t)
            q_t = this.expert_weight(t,:);    % 1 * num_expert
            p_t = this.action_weight(t,:,:);  % num_expert * n_actions
            
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