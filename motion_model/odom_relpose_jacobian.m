function [Hnx, Hx] = odom_relpose_jacobian(nx,x,i,flag)
%
% computes the jacobian of the relative position of nx in the coordinate frame of x
% 
%   Hx = d R(x(3))^T * (nx - x) / dx
%   Hnx = d R(x(3))^T * (nx - x) / nx
%
% INPUT:
%  nx = 3 x num_nx = set of (x,y,Q) points
%   x = 3 x num_x = set of (x,y,Q) points
%
%   i = {1,2,12} = which jacobian to compute
%                  1 = Hnx, 2 = Hx, otherwise both
%
%   flag = [false] = if true returns the jacobians of every nx versus
%                    every x; else assumes num_nx = num_x (or one is scalar)
%                    and returns the corresponding jacobians
%
% OUTPUT:
%   Hnx  = (3 x 3 x num_nx x num_x)   = if flag == true
%        = (3 x 3 x max(num_nx,num_x)) = if flag == false
%
%   Hx  = (3 x 3 x num_nx x num_x)   = if flag == true
%       = (max(num_nx,num_x) x 3 x 3) = if flag == false

if( nargin < 4 )
    flag = false;
end

if( nargin < 3 )
    i = 12;
end

num_nx = size(nx,2);
num_x = size(x,2);

if( flag )
    s = sin(x(3,:));         % 1 x num_x
    s = s(ones(num_nx,1),:); % num_nx x num_x
    c = cos(x(3,:));         % 1 x num_x
    c = c(ones(num_nx,1),:); % num_nx x num_x
    
    if( i == 2 )
       Hnx = [];
    else
       Hnx = zeros(3,3,num_nx,num_x);
       Hnx(1,1,:,:) = c;
       Hnx(1,2,:,:) = s;
       Hnx(2,1,:,:) = -s;
       Hnx(2,2,:,:) = c;
       Hnx(3,3,:,:) = 1;
    end
    
    if( i == 1 )
        Hx = [];
    else
        dxx = bsxfun(@minus,transpose(nx(1,:)),x(1,:)); % num_nx x num_x
        dxy = bsxfun(@minus,transpose(nx(2,:)),x(2,:)); % num_nx x num_x        
        Hx = zeros(3,3,num_nx,num_x);
        Hx(1,1,:,:) = -c;
        Hx(1,2,:,:) = -s;
        Hx(1,3,:,:) = -s.*dxx + c.*dxy;
        Hx(2,1,:,:) = s;
        Hx(2,2,:,:) = -c;
        Hx(2,3,:,:) = -c.*dxx -s.*dxy;        
        Hx(3,3,:,:) = -1;
    end    
else
    s = sin(x(3,:));
    c = cos(x(3,:));
    
    if( i == 2 )
        Hnx = [];
    else
        Hnx = zeros(3,3,max(num_nx,num_x));
        Hnx(1,1,:) = c;
        Hnx(1,2,:) = s;
        Hnx(2,1,:) = -s;
        Hnx(2,2,:) = c;
        Hnx(3,3,:) = 1;
    end
    
    if( i == 1 )
        Hx = [];
    else
        dxx = nx(1,:) - x(1,:);
        dxy = nx(2,:) - x(2,:);
        Hx = zeros(3,3,max(num_nx,num_x));
        Hx(1,1,:) = -c;
        Hx(1,2,:) = -s;
        Hx(1,3,:) = -s.*dxx + c.*dxy;
        Hx(2,1,:) = s;
        Hx(2,2,:) = -c;
        Hx(2,3,:) = -c.*dxx -s.*dxy;        
        Hx(3,3,:) = -1;        
    end
end

end