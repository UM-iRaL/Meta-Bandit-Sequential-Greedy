function obj_sum = objective_function(robot_states, target_states, r_senses, fovs)
% param robot_states: 3 x num_rob
% param target_states: 2 x num_tar
% param r_senses: 1 x num_rob
% param fovs: 1 x num_rob
% return obj: objective function
    if size(robot_states, 1) ~= 3 || size(target_states, 1) ~= 2
        error('dimensions mismatch');
    end
    num_tar = size(target_states, 2);
    num_rob = size(robot_states, 2);
    map_size = 500;
    obj = (-1 / map_size^(2)) * ones(num_tar, 1);

    if num_rob ~= 0
        for i = 1:num_tar
            target_state = target_states(:, i);
            for j = 1:num_rob            
                % check if robot j can see this target
                robot_state = robot_states(:, j);    
                if visibility(robot_state, target_state, r_senses(j), fovs(j))
                %if true
                    % target i is observed by robot j
                    d = norm(robot_state(1:2) - target_state);
                    obj(i) = obj(i) - 1 / ((d-5)^(2)+1);

%                     obj(i) = obj(i) * 1 / (d+1);
                    %obj(i) = obj(i) - d;
                end
            end
        end
    end
    
    obj = 1 ./ obj;
    %obj = - 1 ./ obj;

    
    obj_sum = sum(obj);
end


% function d = reward_function(obj_tg)
%     % obj_tg: num_tg x 1
%     d = 0;
%     map_size = 200;
%     num_tg = size(obj_tg, 1);
%     for kk = 1:num_tg
%         if obj_tg(kk) == 0 % meaning this target has not been detected by one single robot
%             obj_tg(kk) = -1/(2*map_size^2);
%         end
%         d = d + 1/obj_tg(kk);
%     end
%     %f(empty)
%     obj_empty = -(2*map_size^2)*num_tg;
%     %f(A_i)
%     d = (d - obj_empty) / (0 - obj_empty);
% end