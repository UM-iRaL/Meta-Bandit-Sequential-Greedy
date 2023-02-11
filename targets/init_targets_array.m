function targets_array = init_targets_array(num_tg, type_tg, v_tg, tg_position, yaw_tg, run_len, motion_tg, dT_tg)
% Input:
% num_tg: number of targets, scalar
% type_tg: type of targets, "normal", "adversarial"
% v_tg: speed of targets, num_tg x 1;
% tg_position: initial position of targets, (x, y, id), 3 x num_tg
% yaw_tg: yaw of targets, num_tg x 1;
% motion_tg: motion type of targets, num_tg x 1, text
% dT_tg: sampling period of each targets, num_tg x 1
% run_len: time length for each targets
% Output:
% targetsArray: array of targets

if size(v_tg, 1) ~= num_tg || size(yaw_tg, 1) ~= num_tg || size(motion_tg, 1) ~= num_tg ...
   || size(tg_position, 2) ~= num_tg || size(type_tg, 1) ~= num_tg || size(dT_tg, 1) ~= num_tg
    error('dimensions mismatch');
end

for kk = 1 : num_tg
    if strcmp(type_tg(kk), "normal")
        targets_array(kk) = target_v1(v_tg(kk), tg_position(:, kk), yaw_tg(kk), run_len, motion_tg(kk), dT_tg(kk));
    else
        targets_array(kk) = adversarial_target_v1(v_tg(kk), tg_position(:, kk), yaw_tg(kk), run_len, motion_tg(kk), dT_tg(kk));
    end
end
end