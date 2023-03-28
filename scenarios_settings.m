function [x_true_init, tg_true_init, v_robot, r_senses, fovs, v_tg, yaw_tg, motion_tg, human_pred, viz_axis] = scenarios_settings(num_robot, num_tg, type_tg, Horizon, run_len)
% return scenario-specific parameters

if num_robot == 2 && num_tg == 2 && strcmp(type_tg, 'normal')
    v_robot = [1.5; 1]*20;
    r_senses = [150; 100];
    fovs = [deg2rad(74); deg2rad(74)];
    v_tg = [0.6; 0.4]*20;
%     v_tg = [0.8; 0.6]*20;
    yaw_tg = [deg2rad(90); deg2rad(0)];
    motion_tg = ["straight"; "straight"];
    x_true_init = zeros(num_robot,3);
    x_true_init(1, :) = [-50;-110; pi/2];
    x_true_init(2, :) = [50; -110; pi/2];
    tg_true_init = zeros(3, num_tg);
    tg_true_init(:, 1) = [-40;-90;1];
    tg_true_init(:, 2) = [ 40;-90;2]; 
    human_pred = zeros(2, num_robot, run_len);
    human_pred(:,:,1) = tg_true_init(1:2, :);
    deviate_rad = deg2rad(30); % deviated angle between human pred and targets
    for t = 2:run_len
        for kk = 1:num_tg
            human_pred(:, kk, t) = human_pred(:, kk, t-1) + v_tg(kk)*Horizon/run_len*[cos(yaw_tg(kk)+deviate_rad);sin(yaw_tg(kk)+deviate_rad)];
        end
    end
    viz_axis = [-700,950,-200,1400];
elseif num_robot == 2 && num_tg == 2 && strcmp(type_tg, 'adversarial')

elseif num_robot == 2 && num_tg == 3 && strcmp(type_tg, 'normal')
%     v_robot = [1.3; 1.1]*20;
%     r_senses = [150; 100];
%     fovs = [deg2rad(64); deg2rad(94)];
%     v_tg = [0.3; 0.5; 0.72]*20;
%     yaw_tg = [0; deg2rad(90); deg2rad(-90)];
%     motion_tg = ["straight";"straight";"circle"];
%     x_true_init = zeros(num_robot,3);
%     x_true_init(1, :) = [-150;0;0];
%     x_true_init(2, :) = [0; -110; pi/2];
%     tg_true_init = zeros(3, num_tg);
%     tg_true_init(:, 1) = [-90;0;1];
%     tg_true_init(:, 2) = [0;-120;2];
%     tg_true_init(:, 3) = [-80;0;3];

    v_robot = [1.4; 1.4]*20;
    r_senses = [150; 150];
    fovs = [deg2rad(64); deg2rad(64)];
    v_tg = [0.5; 0.72; 0.72]*20;
    yaw_tg = [0; deg2rad(60); deg2rad(70)];
    motion_tg = ["straight";"straight";"straight"];
    x_true_init = zeros(num_robot,3);
    x_true_init(1, :) = [-150;0;0];
    x_true_init(2, :) = [120; 0; 0];
    tg_true_init = zeros(3, num_tg);
    tg_true_init(:, 1) = [190;0;1];
    tg_true_init(:, 2) = [-80;0;2];
    tg_true_init(:, 3) = [-80;0;3];
    % pre-defined human advice
    human_pred = zeros(2, num_robot, run_len);
    human_pred(:,1,1) = tg_true_init(1:2, 2);
    num_seg = 5;
    seg_points_1 = zeros(2, num_seg);
    current_track_1 = 2;
    for i = 1:num_seg
        seg_points_1(:, i) =  v_tg(current_track_1)*Horizon* i/num_seg * [cos(yaw_tg(current_track_1)); sin(yaw_tg(current_track_1))];
        if current_track_1 == 2
            current_track_1 = 3;
        else
            current_track_1 = 2;
        end
    end
    dv = zeros(2, num_seg);
    for i = 1 : num_seg
        if i == 1
            dv(:, i) = (seg_points_1(:, 1) - zeros(2, 1)) / (Horizon / num_seg); 
        else
            dv(:, i) = (seg_points_1(:, i) - seg_points_1(:, i-1)) / (Horizon / num_seg); 
        end
    end
    for t = 2:run_len
        current_seg = floor(t/(run_len/num_seg))+1;
        if current_seg > num_seg
            human_pred(:, 1, t) = human_pred(:, 1, t-1) + dv(:, end)*Horizon/run_len;
            break;
        end
        human_pred(:, 1, t) = human_pred(:, 1, t-1) + dv(:, current_seg)*Horizon/run_len;
    end
    
    human_pred(:, 2, 1) = tg_true_init(1:2, 1);
    for t = 2:run_len
        human_pred(:, 2, t) = human_pred(:, 2, t-1) + v_tg(1)*Horizon/run_len*[cos(yaw_tg(1)); sin(yaw_tg(1))]; 
    end
    viz_axis = [-200,1400,-200,1400];
elseif num_robot == 2 && num_tg == 3 && strcmp(type_tg, 'adversarial')

elseif num_robot == 2 && num_tg == 4 && strcmp(type_tg, 'normal')
    v_robot = [1.5; 1.5]*20;
    r_senses = [150; 150];
    fovs = [deg2rad(64); deg2rad(64)];
    v_tg = [0.55; 0.55; 0.55; 0.55]*20;
    yaw_tg = [deg2rad(60); deg2rad(75); deg2rad(0); deg2rad(15)];
    motion_tg = ["straight";"straight";"straight"; "straight"];
    x_true_init = zeros(num_robot,3);
    x_true_init(1, :) = [-150;0;0];
    x_true_init(2, :) = [120; 0; 0];
    tg_true_init = zeros(3, num_tg);

    tg_true_init(:, 1) = [-80;0;2];
    tg_true_init(:, 2) = [-80;0;3];
    tg_true_init(:, 3) = [190;0;1];
    tg_true_init(:, 4) = [190;0;1];

    % pre-defined human advice
    human_pred = zeros(2, num_robot, run_len);
    human_pred(:,1,1) = x_true_init(1, 1:2);
    num_seg = 5;
    seg_points_1 = zeros(2, num_seg);
    current_track_1 = 1;
    for i = 1:num_seg
        seg_points_1(:, i) =  v_tg(current_track_1)*Horizon* i/num_seg * [cos(yaw_tg(current_track_1)); sin(yaw_tg(current_track_1))] + tg_true_init(1:2, current_track_1);
        if current_track_1 == 1
            current_track_1 = 2;
        else
            current_track_1 = 1;
        end
    end
    dv = zeros(2, num_seg);
    for i = 1 : num_seg
        if i == 1
            dv(:, i) = (seg_points_1(:, 1) - human_pred(:, 1, 1)) / (Horizon / num_seg); 
        else
            dv(:, i) = (seg_points_1(:, i) - seg_points_1(:, i-1)) / (Horizon / num_seg); 
        end
    end
    for t = 2:run_len
        current_seg = floor(t/(run_len/num_seg))+1;
        if current_seg > num_seg
            human_pred(:, 1, t) = human_pred(:, 1, t-1) + dv(:, end)*Horizon/run_len;
            break;
        end
        human_pred(:, 1, t) = human_pred(:, 1, t-1) + dv(:, current_seg)*Horizon/run_len;
    end
    
    human_pred(:, 2, 1) = x_true_init(2, 1:2);
    num_seg = 5;
    seg_points_2 = zeros(2, num_seg);
    current_track_2 = 3;
    for i = 1:num_seg
        seg_points_2(:, i) =  v_tg(current_track_2)*Horizon* i/num_seg * [cos(yaw_tg(current_track_2)); sin(yaw_tg(current_track_2))] + tg_true_init(1:2, current_track_2);
        if current_track_2 == 3
            current_track_2 = 4;
        else
            current_track_2 = 3;
        end
    end
    dv = zeros(2, num_seg);
    for i = 1 : num_seg
        if i == 1
            dv(:, i) = (seg_points_2(:, 1) - human_pred(:, 2, 1)) / (Horizon / num_seg); 
        else
            dv(:, i) = (seg_points_2(:, i) - seg_points_2(:, i-1)) / (Horizon / num_seg); 
        end
    end

    for t = 2:run_len
        current_seg = floor(t/(run_len/num_seg))+1;
        if current_seg > num_seg
            human_pred(:, 2, t) = human_pred(:, 2, t-1) + dv(:, end)*Horizon/run_len;
            break;
        end
        human_pred(:, 2, t) = human_pred(:, 2, t-1) + dv(:, current_seg)*Horizon/run_len;
    end
    viz_axis = [-200,1400,-200,1400];
elseif num_robot == 2 && num_tg == 4 && strcmp(type_tg, 'adversarial')

elseif num_robot == 3 && num_tg == 3 && strcmp(type_tg, 'normal')
    v_robot = [1.3; 1.1; 1.3]*20;
    r_senses = [150; 100; 150];
    fovs = [deg2rad(64); deg2rad(94); deg2rad(64)];
    v_tg = [0.3; 0.5; 0.72]*20;
    yaw_tg = [0; deg2rad(90); deg2rad(-90)];
    motion_tg = ["straight";"straight";"circle"];
    x_true_init = zeros(num_robot,3);
    x_true_init(1, :) = [-150;0;0];
    x_true_init(2, :) = [0; -110; pi/2];
    x_true_init(3, :) = [-150; -110; pi/2];
    tg_true_init = zeros(3, num_tg);
    tg_true_init(:, 1) = [-90;0;1];
    tg_true_init(:, 2) = [0;-120;2];
    tg_true_init(:, 3) = [-80;0;3];
end