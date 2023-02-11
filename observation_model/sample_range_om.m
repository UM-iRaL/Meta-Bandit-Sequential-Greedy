function z = sample_range_om(y,x,r_sigma,flag)
%
% y = num_y x num_dim
% x = num_x x num_dim
%
% flag = true = returns z for every combination of y,x
%      = false = assumes num_y = num_x (or some are scalar) and 
%                returns a vector z
%
% OUTPUT:
%   z in [0, inf]   = num_y x num_x            = if flag == true
%                   = max(num_y,num_x) x 1     = if flag == false
%
%

if(nargin<4)
    flag = false;
end

if(flag)
    r = range_nx(x(:,1),x(:,2),y(:,1),y(:,2),flag);      % num_y x num_x
else
    r = range_nx(x(:,1),x(:,2),y(:,1),y(:,2));           % max(num_y,num_x) x 1
end


z = max(0,r+r_sigma*randn(size(r)));
end