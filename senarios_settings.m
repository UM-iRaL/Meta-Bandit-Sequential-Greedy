function [x_true_init, tg_true_init, v_robot, r_senses, fovs, v_tg, yaw_tg, motion_tg] = senarios_settings(num_robot, num_tg, type_tg)
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
    tg_true_init(:, 1) = [-20;-90;1];
    tg_true_init(:, 2) = [ 20;-90;2];    
elseif num_robot == 2 && num_tg == 2 && strcmp(type_tg, 'adversarial')

elseif num_robot == 2 && num_tg == 3 && strcmp(type_tg, 'normal')
    v_robot = [1.3; 1.1]*20;
    r_senses = [150; 100];
    fovs = [deg2rad(64); deg2rad(94)];
    v_tg = [0.3; 0.5; 0.72]*20;
    yaw_tg = [0; deg2rad(90); deg2rad(-90)];
    motion_tg = ["straight";"straight";"circle"];
    x_true_init = zeros(num_robot,3);
    x_true_init(1, :) = [-150;0;0];
    x_true_init(2, :) = [0; -110; pi/2];
    tg_true_init = zeros(3, num_tg);
    tg_true_init(:, 1) = [-90;0;1];
    tg_true_init(:, 2) = [0;-120;2];
    tg_true_init(:, 3) = [-80;0;3];
elseif num_robot == 2 && num_tg == 3 && strcmp(type_tg, 'adversarial')

elseif num_robot == 2 && num_tg == 4 && strcmp(type_tg, 'normal')

elseif num_robot == 2 && num_tg == 4 && strcmp(type_tg, 'adversarial')

end