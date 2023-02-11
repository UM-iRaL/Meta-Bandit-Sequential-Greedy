function y = inverse_rb(x,z,flag)
% INPUT:
%   x = num_x x 3 = (x,y,orientation)
%   z = num_z x 2 = (range, bearing)
%
%   flag = if true returns the inverse measurement of every z versus every x
%          else assumes num_x = num_z (or one is scalar) and returns the 
%          inverse measurement for the matching points
%
% OUTPUT:
%   y = num_x x num_z x 2 = if flag == true
%     = max(num_x,num_z) x 2 = if flag == false
%
%
if( nargin < 3 )
    flag = false;
end

num_z = size(z,1);
num_x = size(x,1);
if(flag)
    y = zeros(num_x,num_z,2);
    dyaw = bsxfun(@plus, x(:,3), transpose(z(:,2))); % num_x x num_z
    y(:,:,1) = bsxfun(@plus,x(:,1),bsxfun(@times,transpose(z(:,1)),cos(dyaw)));
    y(:,:,2) = bsxfun(@plus,x(:,2),bsxfun(@times,transpose(z(:,1)),sin(dyaw)));
else
    y = zeros(max(num_z,num_x), 2);
    y(:,1) = x(:,1) + z(:,1) .* cos(x(:,3) + z(:,2));
    y(:,2) = x(:,2) + z(:,1) .* sin(x(:,3) + z(:,2));
end


end
