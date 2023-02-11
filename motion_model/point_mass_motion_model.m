function new_x = point_mass_motion_model(x, u, samp)
% Input: 
% x = 3 x 1
% u = 2 x num_u, (vx, vy)'
% Output:
    
    num_u = size(u, 2);

    dx = zeros(1, num_u);
    dy = zeros(1, num_u);
    dx = u(1, :) * samp;
    dy = u(2, :) * samp;

    yaw = atan2(dy, dx);
    nx = size(x, 1);
    
    new_x = zeros(nx, num_u);
    
    new_x(1, :) = x(1) + dx;
    new_x(2, :) = x(2) + dy;
    new_x(3, :) = restrict_angle(yaw);

end