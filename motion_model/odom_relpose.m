function dx = odom_relpose(nx,x,flag)
%
% computes the relative position of nx in the coordinate frame of x
%   dx = inv(w_T_1)*nx_w = R(x(3))^T * (nx - x) 
%       w_T_1 = transformation from x to world: R(x(3)) and [x(1),x(2)]
%
% INPUT:
%  nx = 3 x num_nx = set of (x,y,Q) points
%   x = 3 x num_x = set of (x,y,Q) points
%
%   flag = [false] = if true returns the displacement of every nx versus
%                    every x else assumes num_nx = num_x (or one is scalar)
%                   and returns the corresponding displacements
%
% OUTPUT:
%   dx  = (3 x num_nx x num_x)   = if flag == true
%       = (3 x max(num_nx,num_x)) = if flag == false
%


if( nargin < 3 )
    flag = false;
end

num_nx = size(nx,2);
num_x = size(x,2);

if( flag )
    dx = zeros(3,num_nx,num_x);
    dxx = bsxfun(@minus,transpose(nx(1,:)),x(1,:)); % num_nx x num_x
    dxy = bsxfun(@minus,transpose(nx(2,:)),x(2,:)); % num_nx x num_x
    dx(3,:,:) = restrict_angle(bsxfun(@minus,transpose(nx(3,:)),x(3,:))); % num_nx x num_x
    
    s = sin(x(3,:));    % 1 x num_x
    c = cos(x(3,:));    % 1 x num_x
    
    dx(1,:,:) = bsxfun(@times,dxx,c) + bsxfun(@times,dxy,s);
    dx(2,:,:) = -bsxfun(@times,dxx,s) + bsxfun(@times,dxy,c);
else
    dx = zeros(3,max(num_nx,num_x));
    dx(3,:) = restrict_angle(nx(3,:)-x(3,:));
    dxx = nx(1,:) - x(1,:);
    dxy = nx(2,:) - x(2,:);
    s = sin(x(3,:));
    c = cos(x(3,:));
    
    dx(1,:) = dxx.*c + dxy.*s;
    dx(2,:) = -dxx.*s + dxy.*c;
end



end
