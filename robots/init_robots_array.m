function robots_array = init_robots_array(num_robot, x_position, r_senses, fovs, dTs)
% Input:
% num_robot: number of robots, scalar
% x_position: initial position of robots,  num_robot x 3
% r_senses: sensing range of each robot, num_robot x 1
% fovs: field of views of each robot, num_robot x 1
% dTs: sampling period of robots, num_robot x 1
% Output:
% robots_array: array of robot objects

if size(r_senses, 1) ~= num_robot || size(fovs, 1) ~= num_robot || size(x_position, 1) ~= num_robot 
    error('dimension mismatch');
end

for r = 1:num_robot
    if dTs(r) <= 0
        error('sampling period cannnot be negative.');
    end
    if dTs/dTs(1) ~= ones(r,1)
        error('we do not support different sampling time currently.')
    end
end

for r = 1:num_robot
    robots_array(r) = robot_nx(x_position(r, :), r_senses(r), fovs(r), dTs(r));
end

end