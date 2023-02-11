function r = range_nx(xx,xy,yx,yy,flag)
% r = range_nx(xx,xy,yx,yy,flag)
%
% returns the distnace from (xx,xy) to (yx,yy)
%
%   xx = num_x x 1
%   xy = num_x x 1
%   yx = num_y x 1
%   yy = num_y x 1
%   flag = if true returns the range of every x versus every y
%          else assumes num_x = num_y and returns the range for the
%          matching points
%
% OUTPUT:
%   r in [0, inf]  = num_y x num_x = if flag == true
%                  = max(num_x,num_y) x 1 = if flag == false
%

if(nargin < 5)
    flag = false;
end

if(flag)
    r = sqrt(bsxfun(@minus,yy,xy.').^2 + bsxfun(@minus,yx,xx.').^2);
else
    r = sqrt((yy-xy).^2 + (yx-xx).^2);
end

end