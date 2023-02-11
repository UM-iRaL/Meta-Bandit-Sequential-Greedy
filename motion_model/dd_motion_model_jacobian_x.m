function F = dd_motion_model_jacobian_x(x,U,samp,flag)
% jacobian of motion model wrt x
%
% @input:
%   x = 3 x num_x = starting states (x,y,Q)
%   U = 2 x num_u = [v(1), ..., v(n);
%                    w(1), ..., w(n)];
%   samp = time discretization step
%
%   flag = [false] = if true returns the new state of every x versus every u
%                   else assumes num_sta = num_controls (or one is scalar)
%                   and returns the corresponding new states
%
% @output:
%   F = (3 x 3 x num_u x num_x)   = if flag == true
%     = (3 x 3 x max(num_u,num_x)) = if flag == false
%

if( nargin < 4 )
    flag = false;
end

% discretization step
if( nargin < 3)
    samp = 0.5;
end

[~, num_ctrl] = size(U);
[x_dim, num_sta] = size(x);


if( flag )
    F = zeros(x_dim,x_dim, num_ctrl,num_sta);
    
    dyaw = transpose(samp*U(2,:));  % num_ctrl x 1
    R = transpose(U(1,:)./U(2,:));  % num_ctrl x 1

    new_x_3 = bsxfun(@plus, x(3,:),dyaw);   % num_ctrl x num_sta


    % Probabilistic Robotics eq (7.8)
    dx = bsxfun(@times, R, bsxfun(@minus,cos(new_x_3),cos(x(3,:)))); 
    dy = bsxfun(@times, R, bsxfun(@minus,sin(new_x_3),sin(x(3,:))));
    

    isSmall_w = (abs(dyaw) <= 0.001);
    if(any(isSmall_w))
        tmp = bsxfun(@plus,x(3,:),dyaw(isSmall_w)/2);
        dx(isSmall_w,:) = bsxfun(@times,samp*U(1,isSmall_w)',-sin(tmp));
        dy(isSmall_w,:) = bsxfun(@times,samp*U(1,isSmall_w)',cos(tmp));
    end

    
    F(1,1,:,:) = 1;
    F(1,2,:,:) = 0;
    F(1,3,:,:) = dx;
    F(2,1,:,:) = 0;
    F(2,2,:,:) = 1;
    F(2,3,:,:) = dy;    
    F(3,1,:,:) = 0;
    F(3,2,:,:) = 0;
    F(3,3,:,:) = 1; 
    

else
    dyaw = samp*U(2,:);  % 1 x num_ctrl
    R = U(1,:)./U(2,:);  % 1 x num_ctrl
    
        
    if( num_ctrl == 1 )
        
        new_x_3 = x(3,:) + dyaw;
        
        if(abs(dyaw) <= 0.001)
            tmp = x(3,:) + dyaw/2;
            dx = -samp*U(1)*sin(tmp);
            dy = samp*U(1)*cos(tmp);
        else
            dx = R*(cos(new_x_3) - cos(x(3,:)));
            dy = R*(-sin(x(3,:)) + sin(new_x_3));
        end
        
        F = zeros(3,3,num_sta);
        F(1,1,:) = 1;
        F(1,2,:) = 0;
        F(1,3,:) = dx;
        F(2,1,:) = 0;
        F(2,2,:) = 1;
        F(2,3,:) = dy;
        F(3,1,:) = 0;
        F(3,2,:) = 0;
        F(3,3,:) = 1;
          
    elseif( num_sta == num_ctrl)
        
        new_x_3 = x(3,:) + dyaw;    % 1 x num_sta
        
        isSmall_w = (abs(dyaw) <= 0.001);   % 1 x num_sta
        dx = zeros(1,num_sta);
        dy = zeros(1,num_sta);
        dx(~isSmall_w) = R(~isSmall_w).*(cos(new_x_3(~isSmall_w)) - cos(x(3,~isSmall_w)));
        dy(~isSmall_w) = R(~isSmall_w).*(-sin(x(3,~isSmall_w)) + sin(new_x_3(~isSmall_w)));
        
        tmp = x(3,isSmall_w)+dyaw(isSmall_w)/2;
        dx(isSmall_w) = -samp*U(1,isSmall_w).*sin(tmp);
        dy(isSmall_w) = samp*U(1,isSmall_w).*cos(tmp);
        
        F = zeros(3,3,num_sta);
        F(1,1,:) = 1;
        F(1,2,:) = 0;
        F(1,3,:) = dx;
        F(2,1,:) = 0;
        F(2,2,:) = 1;
        F(2,3,:) = dy;
        F(3,1,:) = 0;
        F(3,2,:) = 0;
        F(3,3,:) = 1;
        
        
    elseif( num_sta == 1)
        
        new_x_3 = x(3) + dyaw;
        isSmall_w = (abs(dyaw) <= 0.001);   % 1 x num_ctrl
        dx = zeros(1,num_ctrl);
        dy = zeros(1,num_ctrl); 
        dx(~isSmall_w) = R(~isSmall_w).*(cos(new_x_3(~isSmall_w)) - cos(x(3)));
        dy(~isSmall_w) = R(~isSmall_w).*(-sin(x(3)) + sin(new_x_3(~isSmall_w)));
        
        tmp = x(3)+dyaw(isSmall_w)/2;
        dx(isSmall_w) = -samp*U(1,isSmall_w).*sin(tmp);
        dy(isSmall_w) = samp*U(1,isSmall_w).*cos(tmp);
        
        F = zeros(3,3,num_ctrl);
        F(1,1,:) = 1;
        F(1,2,:) = 0;
        F(1,3,:) = dx;
        F(2,1,:) = 0;
        F(2,2,:) = 1;
        F(2,3,:) = dy;
        F(3,1,:) = 0;
        F(3,2,:) = 0;
        F(3,3,:) = 1;

    else
        error('[dd_motion_model] Dimension mismatch!');
    end
end


end
