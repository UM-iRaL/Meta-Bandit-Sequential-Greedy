classdef human_nx < handle
    properties (SetAccess = protected, GetAccess = public)
        memory_len;
        memory_noise;
        predict_len;
        targetsLogger;
        tar_cur;
    end
    
    methods
        function this = human_nx(num_tg, memory_len, memory_noise, predict_len)
            this.memory_len = memory_len;
            this.memory_noise = memory_noise;
            this.predict_len = predict_len;
            for i = 1:num_tg
                this.targetsLogger = [this.targetsLogger; Queue];
            end
            this.tar_cur = zeros(2, num_tg);
        end 

        function memorize(this, t, estm_tg, ids)
            % Input:
            % estm_tg: estimated targets position 2 x num_det
            % ids: corresponding id for targets.
            num_det = size(estm_tg, 2);
            for i = 1:num_det
                cur_tg = ids(i);
                cur_mem_len = this.targetsLogger(cur_tg).getNumElements;
                if cur_mem_len >= this.memory_len
                    this.targetsLogger(cur_tg).remove;
                end
                this.targetsLogger(cur_tg).add([t;estm_tg(:, i)]);
                this.tar_cur(:, i) = estm_tg(:, i);
            end
        end

        function pred = predictBezier(this)
            
            % Output: prediction of targets trajectory
            % pred : 4 x num_tg (id; t; x; y) 
            pred = zeros(4, 0);
            for i = 1:size(this.targetsLogger, 1)
                curr_W = cell2mat(this.targetsLogger(i).elements);  % current waypoints.
                if size(curr_W, 2) == 1 || size(curr_W, 2) == 0 % if we don't have any data we don't do predictions.
                    continue;
                end
                last_time_interval = curr_W(1, end) - curr_W(1, end - 1);
                predict_ratio = (last_time_interval + this.predict_len) / last_time_interval;
                 
                [Bez, ~] =GnBezierFit(curr_W(2:end, :), 3);
                cur_pred = BezierLocal(Bez(end).Q, predict_ratio);
                % patch id to prediction
                cur_pred = [i*ones(1, size(cur_pred, 2));cur_pred];
                pred = [pred cur_pred];
            end
        end
    end
end

function [C] = BezierLocal(Q,t)
%check if rational or regular bezier curve. if no info, assume regular
p=size(Q,2)-1; %degree of bezier curve
%m=size(Q,1);   %dimension of space

%construct bernstein basis matrix
Over = @(u,d) factorial(u)./(factorial(d).*(factorial(u-d)));
Bp=zeros(p+1,p+1);
for kk=0:p
    ii=kk:p;
    Bp(kk+1,ii+1) =  (-1).^(ii-kk).*Over(p,ii).*Over(ii,kk) ; %*t^i
end

%evaluate numerically
Up  = t.^((0:p)');   %time power matrix
C   = Q*Bp*Up;

end