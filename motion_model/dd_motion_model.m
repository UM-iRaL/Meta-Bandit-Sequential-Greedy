function new_x = dd_motion_model(x,U,samp,flag)
% new_x = dd_motion_model(x,U,samp,flag)
%
% updates the vehicle state
%
% @input:
%   x = 3 x num_x = starting states of (x,y,Q)
%   U = 2 x num_u = [v(1), ..., v(n);
%                       w(1), ..., w(n)];
%   samp = time discretization step
%
%   flag = [false] = if true returns the new state of every x versus every u
%                   else assumes num_x = num_u (or one is scalar)
%                   and returns the corresponding new states
%
% @output:
%   new_x = (3 x num_u x num_x)   = if flag == true
%         = (3 x max(num_x,num_u)) = if flag == false
%

if( nargin < 4 )
    flag = false;
end

% discretization step
if( nargin < 3)
    samp = 0.5;
end

num_u = size(U,2);
num_x = size(x,2);


if( flag )
    dyaw = transpose(samp*U(2,:));  % num_u x 1
    R = transpose(U(1,:)./U(2,:));  % num_u x 1

    new_x_3 = bsxfun(@plus, x(3,:),dyaw);   % num_u x num_x


    %dx = R.*(sin(x(3,:)+dyaw)-sin(x(3,:)));    % num_u x num_x
    dx = bsxfun(@times, R, bsxfun(@minus,sin(new_x_3),sin(x(3,:)))); 


    %dy = R.*(cos(x(3)) - cos(x(3)+dyaw));
    dy = bsxfun(@times, R, bsxfun(@minus, cos(x(3,:)),cos(new_x_3)));

    isSmall_w = (abs(dyaw) <= 0.001);
    if(any(isSmall_w))
        tmp = bsxfun(@plus,x(3,:),dyaw(isSmall_w)/2);
        dx(isSmall_w,:) = bsxfun(@times,samp*U(1,isSmall_w)',cos(tmp));
        dy(isSmall_w,:) = bsxfun(@times,samp*U(1,isSmall_w)',sin(tmp));
    end

    %dx(isSmall_w) = samp*U(1,isSmall_w).*cos(x(3) + samp*U(2,isSmall_w)/2);
    %dy(isSmall_w) = samp*U(1,isSmall_w).*sin(x(3) + samp*U(2,isSmall_w)/2);

    %new_x = [x(1) + dx; x(2) + dy; restrict_angle(x(3) + dyaw)];
    nx = size(x,1);
    
    new_x = zeros(nx, num_u, num_x);
    new_x(1,:,:) = bsxfun(@plus,x(1,:),dx);
    new_x(2,:,:) = bsxfun(@plus,x(2,:),dy);
    new_x(3,:,:) = restrict_angle(new_x_3);
    
    if nx > 3
        new_x(4,:,:) = repmat(samp*U(1,:),num_x,1)';
        new_x(5,:,:) = repmat(samp*U(2,:),num_x,1)';
    end


    % new_1 = x(1) + dx;
    % new_2 = x(2) + dy;
    % new_3 = restrict_angle(x(3) + dyaw);
    % 
    % new_x = transpose([new_1(:), new_2(:), new_3(:)]);
else
    dyaw = samp*U(2,:);  % 1 x num_u
    R = U(1,:)./U(2,:);  % 1 x num_u
    
        
    if( num_u == 1 )
        
        new_x_3 = x(3,:) + dyaw;
        
        if(abs(dyaw) <= 0.001)
            tmp = x(3,:) + dyaw/2;
            dx = samp*U(1)*cos(tmp);
            dy = samp*U(1)*sin(tmp);
        else
            dx = R*(sin(new_x_3) - sin(x(3,:)));
            dy = R*(cos(x(3,:)) - cos(new_x_3));
        end
        
        nx = size(x,1);
        new_x = zeros(nx, num_x);
        new_x(1,:) = x(1,:)+dx;
        new_x(2,:) = x(2,:)+dy;
        new_x(3,:) = restrict_angle(new_x_3);        
        
        if nx > 3
            new_x(4,:) = samp*U(1);
            new_x(5,:) = samp*U(2);
        end        
    elseif( num_x == num_u)
        
        new_x_3 = x(3,:) + dyaw;    % 1 x num_x
        
        isSmall_w = (abs(dyaw) <= 0.001);   % 1 x num_x
        dx = zeros(1,num_x);
        dy = zeros(1,num_x);
        dx(~isSmall_w) = R(~isSmall_w).*(sin(new_x_3(~isSmall_w)) - sin(x(3,~isSmall_w)));
        dy(~isSmall_w) = R(~isSmall_w).*(cos(x(3,~isSmall_w)) - cos(new_x_3(~isSmall_w)));
        
        tmp = x(3,isSmall_w)+dyaw(isSmall_w)/2;
        dx(isSmall_w) = samp*U(1,isSmall_w).*cos(tmp);
        dy(isSmall_w) = samp*U(1,isSmall_w).*sin(tmp);
        
        
        nx = size(x,1);
        new_x = zeros(nx, num_x);
        new_x(1,:) = x(1,:)+dx;
        new_x(2,:) = x(2,:)+dy;
        new_x(3,:) = restrict_angle(new_x_3);

        if nx > 3
            new_x(4,:) = samp*U(1,:);
            new_x(5,:) = samp*U(2,:);
        end
        
    elseif( num_x == 1)
        
        new_x_3 = x(3) + dyaw;
        isSmall_w = (abs(dyaw) <= 0.001);   % 1 x num_u
        dx = zeros(1,num_u);
        dy = zeros(1,num_u); 
        dx(~isSmall_w) = R(~isSmall_w).*(sin(new_x_3(~isSmall_w)) - sin(x(3)));
        dy(~isSmall_w) = R(~isSmall_w).*(cos(x(3)) - cos(new_x_3(~isSmall_w)));
        
        tmp = x(3)+dyaw(isSmall_w)/2;
        dx(isSmall_w) = samp*U(1,isSmall_w).*cos(tmp);
        dy(isSmall_w) = samp*U(1,isSmall_w).*sin(tmp);
        
        nx = size(x,1);
        new_x = zeros(nx, num_u);
        new_x(1,:) = x(1)+dx;
        new_x(2,:) = x(2)+dy;
        new_x(3,:) = restrict_angle(new_x_3);
        
        if nx > 3
            new_x(4,:) = samp*U(1,:);
            new_x(5,:) = samp*U(2,:);
        end

    else
        error('[dd_motion_model] Dimension mismatch!');
    end
end


end
